#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <json/json.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/mesh_utils.hpp"
#include "panda_arm_control/pose_utils.hpp"
#include "panda_arm_control/potential_field_sampler.hpp"
#include "panda_arm_control/rkga_scp.hpp"
#include "panda_arm_control/viewpoint_types.hpp"

namespace
{

struct Params
{
	std::string mesh_path;
	double mesh_scale = 0.01;
	int target_faces = 1000;
	int n_surface_samples = 500;
	std::vector<double> standoff_distances = {0.01, 0.02, 0.03};
	std::vector<double> tilt_angles_deg = {0.0, 15.0, -15.0};
	double min_clearance = 0.005;
	double fov_deg = 30.0;
	double max_distance = 20.0;
	double angle_threshold_deg = 70.0;
	int max_rays_per_view = 2000;
	double target_area_visibility = 0.95;
	double min_new_area_ratio = 0.001;
	double ik_timeout = 0.1;
	int random_seed = 42;
	std::string group_name = "panda_arm";
	std::vector<double> object_translation_world = {0.2, 0.2, 0.38};
	std::vector<double> object_rotation_rpy_deg = {0.0, 0.0, 0.0};
	std::vector<double> t_tcp_camera_xyz = {0.0, 0.0, 0.0};
	std::vector<double> t_tcp_camera_quat_xyzw = {0.0, 0.0, 0.0, 1.0};
	// Tour ordering cost is euclidean_distance_m + joint_distance_weight * joint_distance_rad,
	// so this trades off "prefer short Cartesian hops" against "prefer small joint motion"
	// (1.0 rad of joint motion counts as much as joint_distance_weight meters of TCP travel).
	double joint_distance_weight = 0.1;
	// Applied only to the original mesh's comparison-marker position (not the object's actual
	// pose), so the original and simplified meshes render side by side instead of overlapping.
	std::vector<double> mesh_comparison_offset = {0.0, 0.3, 0.0};
	// Swaps candidate generation/reachability/selection for the potential-field sampler +
	// collision-aware IK + RKGA-SCP genetic algorithm (rkga_scp.hpp) instead of the standoff/tilt
	// grid + plain IK + greedy set-cover + NN/2-opt pipeline.
	bool use_rkga = false;
	// Only used when use_rkga is true. Instead of SolveRkgaScp's GA jointly picking selection +
	// order, select via the same GreedySelectViewpoints used by the non-RKGA pipeline, then order
	// via NearestNeighborOrderMatrix + TwoOptImproveMatrix -- using the real travel_cost_matrix
	// costs, not TourCost's straight-line estimate. Lets GA-ordering vs NN/2-opt-ordering be
	// compared under the exact same real-cost model.
	bool use_matrix_2opt = false;
	// Per-pair timeout for ComputeEndEffectorTravelDistanceMatrix's real motion-planning calls
	// (used only when use_rkga is true). Runs once, up front, for every pair of reachable
	// candidates -- keep this well under panda_arm_control's 20s execution-time budget, since the
	// total cost is this value times roughly N^2/2 pairs.
	double rkga_planning_time = 2.0;
	std::string output_dir = "/tmp/viewpoint_planner_output";
};

Eigen::Vector3d ToVector3(const std::vector<double>& v, const Eigen::Vector3d& fallback)
{
	if (v.size() != 3)
		return fallback;
	return Eigen::Vector3d(v[0], v[1], v[2]);
}

Eigen::Matrix3d RotationFromRpyDeg(const std::vector<double>& rpy_deg)
{
	if (rpy_deg.size() != 3)
		return Eigen::Matrix3d::Identity();

	double roll = rpy_deg[0] * M_PI / 180.0;
	double pitch = rpy_deg[1] * M_PI / 180.0;
	double yaw = rpy_deg[2] * M_PI / 180.0;

	// Matches scipy's R.from_euler("xyz", [roll, pitch, yaw]).as_matrix(): Rz * Ry * Rx.
	Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
						 .toRotationMatrix();
	return R;
}

Eigen::Isometry3d IsometryFromXyzQuat(const std::vector<double>& xyz, const std::vector<double>& quat_xyzw)
{
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	if (xyz.size() == 3)
		T.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
	if (quat_xyzw.size() == 4)
		T.linear() = Eigen::Quaterniond(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]).toRotationMatrix();
	return T;
}

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& v)
{
	geometry_msgs::msg::Point p;
	p.x = v.x();
	p.y = v.y();
	p.z = v.z();
	return p;
}

} // namespace

class ViewpointPlannerNode : public rclcpp::Node
{
public:
	explicit ViewpointPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
		: Node("viewpoint_planner", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
	{
	}

	void init()
	{
		declareToolParameters();
		loadParameters();

		robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
		robot_model_ = loader.getModel();
		robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
		robot_state_->setToDefaultValues();
		jmg_ = robot_state_->getJointModelGroup(params_.group_name);

		// Capture the home tool0 position/joint values now, before computeReachability() mutates
		// robot_state_ with IK solutions -- this is the reference point the nearest-neighbor
		// tour starts from.
		robot_state_->update();
		home_tcp_position_ = robot_state_->getGlobalLinkTransform("tool0").translation();
		robot_state_->copyJointGroupPositions(jmg_, home_joint_values_);

		marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
			"/viewpoint_markers", rclcpp::QoS(1).transient_local());

		mesh_comparison_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
			"/mesh_comparison_markers", rclcpp::QoS(1).transient_local());

		waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
			"/cartesian_waypoints", rclcpp::QoS(1).transient_local());

		runPipeline();

		marker_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { marker_pub_->publish(marker_array_); });
	}

private:
	Params params_;

	moveit::core::RobotModelPtr robot_model_;
	moveit::core::RobotStatePtr robot_state_;
	const moveit::core::JointModelGroup* jmg_ = nullptr;
	Eigen::Vector3d home_tcp_position_ = Eigen::Vector3d::Zero();
	std::vector<double> home_joint_values_;

	std::vector<ViewpointCandidate> reachable_candidates_;
	std::vector<ViewpointCandidate> unreachable_visible_;
	std::vector<const ViewpointCandidate*> selected_;
	std::string resolved_mesh_path_;
	std::string resolved_simplified_mesh_path_;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_comparison_pub_;
	rclcpp::TimerBase::SharedPtr marker_timer_;
	visualization_msgs::msg::MarkerArray marker_array_;

	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_pub_;

	// automatically_declare_parameters_from_overrides(true) (needed for robot_description*)
	// already auto-declares any of these that the launch file passed as overrides, so guard
	// against re-declaring them here.
	template <typename T>
	void declareIfNeeded(const std::string& name, const T& default_value)
	{
		if (!has_parameter(name))
			declare_parameter(name, default_value);
	}

	void declareToolParameters()
	{
		declareIfNeeded("mesh_path", params_.mesh_path);
		declareIfNeeded("mesh_scale", params_.mesh_scale);
		declareIfNeeded("target_faces", params_.target_faces);
		declareIfNeeded("n_surface_samples", params_.n_surface_samples);
		declareIfNeeded("standoff_distances", params_.standoff_distances);
		declareIfNeeded("tilt_angles_deg", params_.tilt_angles_deg);
		declareIfNeeded("min_clearance", params_.min_clearance);
		declareIfNeeded("fov_deg", params_.fov_deg);
		declareIfNeeded("max_distance", params_.max_distance);
		declareIfNeeded("angle_threshold_deg", params_.angle_threshold_deg);
		declareIfNeeded("max_rays_per_view", params_.max_rays_per_view);
		declareIfNeeded("target_area_visibility", params_.target_area_visibility);
		declareIfNeeded("min_new_area_ratio", params_.min_new_area_ratio);
		declareIfNeeded("ik_timeout", params_.ik_timeout);
		declareIfNeeded("random_seed", params_.random_seed);
		declareIfNeeded("group_name", params_.group_name);
		declareIfNeeded("object_translation_world", params_.object_translation_world);
		declareIfNeeded("object_rotation_rpy_deg", params_.object_rotation_rpy_deg);
		declareIfNeeded("t_tcp_camera_xyz", params_.t_tcp_camera_xyz);
		declareIfNeeded("t_tcp_camera_quat_xyzw", params_.t_tcp_camera_quat_xyzw);
		declareIfNeeded("joint_distance_weight", params_.joint_distance_weight);
		declareIfNeeded("mesh_comparison_offset", params_.mesh_comparison_offset);
		declareIfNeeded("use_rkga", params_.use_rkga);
		declareIfNeeded("use_matrix_2opt", params_.use_matrix_2opt);
		declareIfNeeded("rkga_planning_time", params_.rkga_planning_time);
		declareIfNeeded("output_dir", params_.output_dir);
	}

	void loadParameters()
	{
		get_parameter("mesh_path", params_.mesh_path);
		get_parameter("mesh_scale", params_.mesh_scale);
		get_parameter("target_faces", params_.target_faces);
		get_parameter("n_surface_samples", params_.n_surface_samples);
		get_parameter("standoff_distances", params_.standoff_distances);
		get_parameter("tilt_angles_deg", params_.tilt_angles_deg);
		get_parameter("min_clearance", params_.min_clearance);
		get_parameter("fov_deg", params_.fov_deg);
		get_parameter("max_distance", params_.max_distance);
		get_parameter("angle_threshold_deg", params_.angle_threshold_deg);
		get_parameter("max_rays_per_view", params_.max_rays_per_view);
		get_parameter("target_area_visibility", params_.target_area_visibility);
		get_parameter("min_new_area_ratio", params_.min_new_area_ratio);
		get_parameter("ik_timeout", params_.ik_timeout);
		get_parameter("random_seed", params_.random_seed);
		get_parameter("group_name", params_.group_name);
		get_parameter("object_translation_world", params_.object_translation_world);
		get_parameter("object_rotation_rpy_deg", params_.object_rotation_rpy_deg);
		get_parameter("t_tcp_camera_xyz", params_.t_tcp_camera_xyz);
		get_parameter("t_tcp_camera_quat_xyzw", params_.t_tcp_camera_quat_xyzw);
		get_parameter("joint_distance_weight", params_.joint_distance_weight);
		get_parameter("mesh_comparison_offset", params_.mesh_comparison_offset);
		get_parameter("use_rkga", params_.use_rkga);
		get_parameter("use_matrix_2opt", params_.use_matrix_2opt);
		get_parameter("rkga_planning_time", params_.rkga_planning_time);
		get_parameter("output_dir", params_.output_dir);
	}

	void runPipeline()
	{
		std::string mesh_path = params_.mesh_path;
		if (mesh_path.empty())
		{
			mesh_path =
				ament_index_cpp::get_package_share_directory("panda_arm_control") + "/meshes/bunny_holding_eggs.stl";
		}

		resolved_mesh_path_ = mesh_path;
		std::filesystem::create_directories(params_.output_dir);

		auto original_poly = LoadAndScaleMesh(mesh_path, params_.mesh_scale);
		MeshData original_mesh = BuildMeshData(original_poly);

		auto simplified_poly = SimplifyMesh(original_poly, params_.target_faces);
		MeshData simplified_mesh = BuildMeshData(simplified_poly);

		resolved_simplified_mesh_path_ = params_.output_dir + "/simplified_mesh.stl";
		ExportMeshToStl(simplified_poly, resolved_simplified_mesh_path_);

		std::mt19937 rng(params_.random_seed);
		std::vector<ViewpointCandidate> candidates;
		if (params_.use_rkga)
		{
			candidates = GeneratePotentialFieldViewCandidates(
				simplified_mesh, params_.n_surface_samples, params_.standoff_distances, rng);
		}
		else
		{
			candidates = GenerateViewCandidates(
				simplified_mesh, params_.n_surface_samples, params_.standoff_distances, params_.tilt_angles_deg, rng);
		}

		RCLCPP_INFO(get_logger(), "candidate views: %zu", candidates.size());

		candidates = FilterByClearance(original_poly, std::move(candidates), params_.min_clearance);

		VisibilityChecker vis_checker(simplified_mesh);
		candidates = ComputeVisibility(
			vis_checker, std::move(candidates), params_.fov_deg, params_.max_distance, params_.angle_threshold_deg,
			params_.max_rays_per_view);

		RCLCPP_INFO(get_logger(), "visible candidate views: %zu", candidates.size());

		planning_scene_monitor::PlanningSceneMonitorPtr rkga_local_scene;
		if (params_.use_rkga)
		{
			Eigen::Vector3d object_translation_world =
				ToVector3(params_.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
			Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(params_.object_rotation_rpy_deg);
			Eigen::Isometry3d t_tcp_camera =
				IsometryFromXyzQuat(params_.t_tcp_camera_xyz, params_.t_tcp_camera_quat_xyzw);

			rkga_local_scene = BuildLocalCollisionScene(
				shared_from_this(), resolved_mesh_path_, params_.mesh_scale, object_translation_world,
				object_rotation_world);

			candidates = ComputeReachabilityWithCollisionCheck(
				std::move(candidates), robot_state_, jmg_, rkga_local_scene, object_translation_world,
				object_rotation_world, t_tcp_camera, params_.ik_timeout);
		}
		else
		{
			candidates = computeReachability(std::move(candidates));
		}

		for (auto& c : candidates)
		{
			if (c.reachable)
				reachable_candidates_.push_back(std::move(c));
			else
				unreachable_visible_.push_back(std::move(c));
		}

		RCLCPP_INFO(
			get_logger(), "reachable candidate views: %zu / %zu", reachable_candidates_.size(),
			reachable_candidates_.size() + unreachable_visible_.size());

		if (params_.use_rkga)
		{
			RCLCPP_INFO(
				get_logger(), "Computing real end-effector travel distance matrix for %zu candidates (this runs "
							  "real motion planning for every pair -- can take a while)...",
				reachable_candidates_.size());

			TravelCostMatrix travel_cost_matrix = ComputeEndEffectorTravelDistanceMatrix(
				shared_from_this(), robot_model_, rkga_local_scene, reachable_candidates_, home_joint_values_,
				params_.group_name, params_.rkga_planning_time);

			if (params_.use_matrix_2opt)
			{
				selected_ = GreedySelectViewpoints(
					simplified_mesh, reachable_candidates_, params_.target_area_visibility,
					params_.min_new_area_ratio);

				RCLCPP_INFO(get_logger(), "selected views: %zu", selected_.size());

				selected_ = NearestNeighborOrderMatrix(
					reachable_candidates_, std::move(selected_), travel_cost_matrix, params_.joint_distance_weight);
				selected_ = TwoOptImproveMatrix(
					reachable_candidates_, std::move(selected_), travel_cost_matrix, params_.joint_distance_weight);

				PrintTourCostBreakdown(reachable_candidates_, selected_, travel_cost_matrix, params_.joint_distance_weight);

				double achieved_visibility = ComputeAreaVisibility(simplified_mesh, selected_);
				RCLCPP_INFO(
					get_logger(),
					"NN+2-opt (real travel_cost_matrix) reordered %zu selected views, %.2f%% area visibility",
					selected_.size(), achieved_visibility * 100.0);
			}
			else
			{
				RkgaScpParams rkga_params;
				rkga_params.target_area_visibility = params_.target_area_visibility;
				rkga_params.random_seed = static_cast<unsigned int>(params_.random_seed);
				rkga_params.joint_distance_weight = params_.joint_distance_weight;

				selected_ = SolveRkgaScp(simplified_mesh, reachable_candidates_, travel_cost_matrix, rkga_params);

				double achieved_visibility = ComputeAreaVisibility(simplified_mesh, selected_);
				RCLCPP_INFO(
					get_logger(), "RKGA-SCP selected %zu views (jointly selected + ordered), %.2f%% area visibility",
					selected_.size(), achieved_visibility * 100.0);
			}
		}
		else
		{
			selected_ = GreedySelectViewpoints(
				simplified_mesh, reachable_candidates_, params_.target_area_visibility, params_.min_new_area_ratio);

			RCLCPP_INFO(get_logger(), "selected views: %zu", selected_.size());

			selected_ = NearestNeighborOrder(
				std::move(selected_), home_tcp_position_, home_joint_values_, params_.joint_distance_weight);
			selected_ = TwoOptImprove(
				std::move(selected_), home_tcp_position_, home_joint_values_, params_.joint_distance_weight);

			RCLCPP_INFO(
				get_logger(), "reordered %zu selected views via nearest-neighbor + 2-opt from robot home position",
				selected_.size());
		}

		exportSelectedViewpoints();
		exportSelectedRobotPoses();
		marker_array_ = buildMarkerArray();
		marker_pub_->publish(marker_array_);

		mesh_comparison_pub_->publish(buildMeshComparisonMarkers());

		publishWaypoints();
	}

	void publishWaypoints()
	{
		if (selected_.empty())
		{
			RCLCPP_WARN(get_logger(), "No reachable/selected viewpoints -- nothing to publish on /cartesian_waypoints");
			return;
		}

		geometry_msgs::msg::PoseArray msg;
		msg.header.frame_id = "world";
		msg.header.stamp = now();
		msg.poses.reserve(selected_.size());
		for (const auto* c : selected_)
			msg.poses.push_back(c->tcp_pose);

		waypoint_pub_->publish(msg);
		RCLCPP_INFO(get_logger(), "Published %zu TCP waypoints to /cartesian_waypoints", msg.poses.size());
	}

	std::vector<ViewpointCandidate> computeReachability(std::vector<ViewpointCandidate> candidates)
	{
		Eigen::Vector3d object_translation_world =
			ToVector3(params_.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(params_.object_rotation_rpy_deg);
		Eigen::Isometry3d t_tcp_camera =
			IsometryFromXyzQuat(params_.t_tcp_camera_xyz, params_.t_tcp_camera_quat_xyzw);

		for (size_t i = 0; i < candidates.size(); ++i)
		{
			auto& c = candidates[i];
			c.tcp_pose = ConvertViewpointToTcpPose(
				c.camera_pos, c.view_dir, object_translation_world, object_rotation_world, t_tcp_camera);

			c.reachable = robot_state_->setFromIK(jmg_, c.tcp_pose, "tool0", params_.ik_timeout);
			if (c.reachable)
				robot_state_->copyJointGroupPositions(jmg_, c.joint_solution);

			if (i % 10 == 0)
				RCLCPP_INFO(get_logger(), "IK check %zu/%zu, reachable=%s", i, candidates.size(), c.reachable ? "yes" : "no");
		}

		return candidates;
	}

	void exportSelectedViewpoints()
	{
		Json::Value root(Json::arrayValue);

		std::string csv_path = params_.output_dir + "/selected_viewpoints.csv";
		std::ofstream csv_file(csv_path);
		csv_file << "id,camera_x,camera_y,camera_z,dir_x,dir_y,dir_z,"
					"target_x,target_y,target_z,distance,num_visible_faces\n";

		for (size_t i = 0; i < selected_.size(); ++i)
		{
			const auto& c = *selected_[i];

			Json::Value item;
			item["id"] = static_cast<int>(i);
			item["camera_pos"]["x"] = c.camera_pos.x();
			item["camera_pos"]["y"] = c.camera_pos.y();
			item["camera_pos"]["z"] = c.camera_pos.z();
			item["view_dir"]["x"] = c.view_dir.x();
			item["view_dir"]["y"] = c.view_dir.y();
			item["view_dir"]["z"] = c.view_dir.z();
			item["target_point"]["x"] = c.target_point.x();
			item["target_point"]["y"] = c.target_point.y();
			item["target_point"]["z"] = c.target_point.z();
			item["distance"] = c.distance;
			item["num_visible_faces"] = c.num_visible_faces;
			root.append(item);

			csv_file << i << "," << c.camera_pos.x() << "," << c.camera_pos.y() << "," << c.camera_pos.z() << ","
					  << c.view_dir.x() << "," << c.view_dir.y() << "," << c.view_dir.z() << "," << c.target_point.x()
					  << "," << c.target_point.y() << "," << c.target_point.z() << "," << c.distance << ","
					  << c.num_visible_faces << "\n";
		}

		std::string json_path = params_.output_dir + "/selected_viewpoints.json";
		std::ofstream json_file(json_path);
		Json::StreamWriterBuilder writer_builder;
		writer_builder["indentation"] = "    ";
		std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
		writer->write(root, &json_file);

		RCLCPP_INFO(get_logger(), "Saved JSON: %s", json_path.c_str());
		RCLCPP_INFO(get_logger(), "Saved CSV: %s", csv_path.c_str());
	}

	void exportSelectedRobotPoses()
	{
		Json::Value root(Json::arrayValue);

		std::string csv_path = params_.output_dir + "/selected_robot_poses.csv";
		std::ofstream csv_file(csv_path);
		csv_file << "id,tcp_x,tcp_y,tcp_z,qx,qy,qz,qw,camera_x,camera_y,camera_z,"
					"view_dir_x,view_dir_y,view_dir_z,num_visible_faces\n";

		for (size_t i = 0; i < selected_.size(); ++i)
		{
			const auto& c = *selected_[i];
			const auto& tcp_pos = c.tcp_pose.position;
			const auto& tcp_quat = c.tcp_pose.orientation;

			Json::Value item;
			item["id"] = static_cast<int>(i);
			item["tcp_position"]["x"] = tcp_pos.x;
			item["tcp_position"]["y"] = tcp_pos.y;
			item["tcp_position"]["z"] = tcp_pos.z;
			item["tcp_quaternion_xyzw"]["x"] = tcp_quat.x;
			item["tcp_quaternion_xyzw"]["y"] = tcp_quat.y;
			item["tcp_quaternion_xyzw"]["z"] = tcp_quat.z;
			item["tcp_quaternion_xyzw"]["w"] = tcp_quat.w;
			item["camera_position"]["x"] = c.camera_pos.x();
			item["camera_position"]["y"] = c.camera_pos.y();
			item["camera_position"]["z"] = c.camera_pos.z();
			item["view_dir"]["x"] = c.view_dir.x();
			item["view_dir"]["y"] = c.view_dir.y();
			item["view_dir"]["z"] = c.view_dir.z();
			item["num_visible_faces"] = c.num_visible_faces;
			root.append(item);

			csv_file << i << "," << tcp_pos.x << "," << tcp_pos.y << "," << tcp_pos.z << "," << tcp_quat.x << ","
					  << tcp_quat.y << "," << tcp_quat.z << "," << tcp_quat.w << "," << c.camera_pos.x() << ","
					  << c.camera_pos.y() << "," << c.camera_pos.z() << "," << c.view_dir.x() << "," << c.view_dir.y()
					  << "," << c.view_dir.z() << "," << c.num_visible_faces << "\n";
		}

		std::string json_path = params_.output_dir + "/selected_robot_poses.json";
		std::ofstream json_file(json_path);
		Json::StreamWriterBuilder writer_builder;
		writer_builder["indentation"] = "    ";
		std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
		writer->write(root, &json_file);

		RCLCPP_INFO(get_logger(), "Saved robot poses JSON: %s", json_path.c_str());
		RCLCPP_INFO(get_logger(), "Saved robot poses CSV: %s", csv_path.c_str());
	}

	visualization_msgs::msg::MarkerArray buildMarkerArray() const
	{
		visualization_msgs::msg::MarkerArray markers;

		Eigen::Vector3d object_translation_world =
			ToVector3(params_.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(params_.object_rotation_rpy_deg);

		const double arrow_length = 0.02;
		int id = 0;
		auto stamp = now();

		// The mesh itself, so it renders alongside the viewpoints (mirrors trimesh's
		// semi-transparent gray preview in visualize_views()).
		Eigen::Quaterniond object_quat(object_rotation_world);
		visualization_msgs::msg::Marker mesh_marker;
		mesh_marker.header.frame_id = "world";
		mesh_marker.header.stamp = stamp;
		mesh_marker.ns = "object_mesh";
		mesh_marker.id = id++;
		mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
		mesh_marker.action = visualization_msgs::msg::Marker::ADD;
		mesh_marker.mesh_resource = "file://" + resolved_mesh_path_;
		mesh_marker.mesh_use_embedded_materials = false;
		mesh_marker.pose.position = ToPoint(object_translation_world);
		mesh_marker.pose.orientation.x = object_quat.x();
		mesh_marker.pose.orientation.y = object_quat.y();
		mesh_marker.pose.orientation.z = object_quat.z();
		mesh_marker.pose.orientation.w = object_quat.w();
		mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = params_.mesh_scale;
		mesh_marker.color.r = 0.7f;
		mesh_marker.color.g = 0.7f;
		mesh_marker.color.b = 0.7f;
		mesh_marker.color.a = 0.5f;
		markers.markers.push_back(mesh_marker);

		auto addCandidateMarkers = [&](const ViewpointCandidate& c, const std::string& ns, float r, float g, float b) {
			Eigen::Vector3d world_pos = object_rotation_world * c.camera_pos + object_translation_world;
			Eigen::Vector3d world_dir = (object_rotation_world * c.view_dir).normalized();

			visualization_msgs::msg::Marker sphere;
			sphere.header.frame_id = "world";
			sphere.header.stamp = stamp;
			sphere.ns = ns + "_camera_pos";
			sphere.id = id++;
			sphere.type = visualization_msgs::msg::Marker::SPHERE;
			sphere.action = visualization_msgs::msg::Marker::ADD;
			sphere.pose.position = ToPoint(world_pos);
			sphere.pose.orientation.w = 1.0;
			sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.006;
			sphere.color.r = r;
			sphere.color.g = g;
			sphere.color.b = b;
			sphere.color.a = 1.0;
			markers.markers.push_back(sphere);

			visualization_msgs::msg::Marker arrow;
			arrow.header.frame_id = "world";
			arrow.header.stamp = stamp;
			arrow.ns = ns + "_view_dir";
			arrow.id = id++;
			arrow.type = visualization_msgs::msg::Marker::ARROW;
			arrow.action = visualization_msgs::msg::Marker::ADD;
			arrow.points.push_back(ToPoint(world_pos));
			arrow.points.push_back(ToPoint(world_pos + world_dir * arrow_length));
			arrow.scale.x = 0.001; // shaft diameter
			arrow.scale.y = 0.003; // head diameter
			arrow.scale.z = 0.0;	// auto head length
			arrow.color.r = r;
			arrow.color.g = g;
			arrow.color.b = b;
			arrow.color.a = 1.0;
			markers.markers.push_back(arrow);
		};

		for (const auto* c : selected_)
			addCandidateMarkers(*c, "selected", 0.0f, 1.0f, 0.0f);

		for (const auto& c : unreachable_visible_)
			addCandidateMarkers(c, "unreachable", 1.0f, 0.5f, 0.0f);

		// Visiting order (nearest-neighbor + 2-opt tour from the robot's home position).
		if (selected_.size() >= 2)
		{
			visualization_msgs::msg::Marker tour;
			tour.header.frame_id = "world";
			tour.header.stamp = stamp;
			tour.ns = "selected_tour";
			tour.id = id++;
			tour.type = visualization_msgs::msg::Marker::LINE_STRIP;
			tour.action = visualization_msgs::msg::Marker::ADD;
			tour.scale.x = 0.0015; // line width
			tour.color.r = 1.0f;
			tour.color.g = 1.0f;
			tour.color.b = 0.0f;
			tour.color.a = 0.8f;

			for (const auto* c : selected_)
			{
				Eigen::Vector3d world_pos = object_rotation_world * c->camera_pos + object_translation_world;
				tour.points.push_back(ToPoint(world_pos));
			}
			markers.markers.push_back(tour);
		}

		return markers;
	}

	// Original mesh (offset by mesh_comparison_offset) and simplified/decimated mesh (at the
	// object's actual pose) side by side, on their own topic separate from /viewpoint_markers.
	visualization_msgs::msg::MarkerArray buildMeshComparisonMarkers() const
	{
		visualization_msgs::msg::MarkerArray markers;

		Eigen::Vector3d object_translation_world =
			ToVector3(params_.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(params_.object_rotation_rpy_deg);
		Eigen::Vector3d comparison_offset =
			ToVector3(params_.mesh_comparison_offset, Eigen::Vector3d(0.0, 0.3, 0.0));

		Eigen::Quaterniond object_quat(object_rotation_world);
		auto stamp = now();
		int id = 0;

		// Original mesh, at the object's real pose (unshifted). resolved_mesh_path_ points at the
		// raw, unscaled STL, so mesh_scale still needs to be applied here.
		visualization_msgs::msg::Marker original_marker;
		original_marker.header.frame_id = "world";
		original_marker.header.stamp = stamp;
		original_marker.ns = "original_mesh";
		original_marker.id = id++;
		original_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
		original_marker.action = visualization_msgs::msg::Marker::ADD;
		original_marker.mesh_resource = "file://" + resolved_mesh_path_;
		original_marker.mesh_use_embedded_materials = false;
		original_marker.pose.position = ToPoint(object_translation_world);
		original_marker.pose.orientation.x = object_quat.x();
		original_marker.pose.orientation.y = object_quat.y();
		original_marker.pose.orientation.z = object_quat.z();
		original_marker.pose.orientation.w = object_quat.w();
		original_marker.scale.x = original_marker.scale.y = original_marker.scale.z = params_.mesh_scale;
		original_marker.color.r = 0.7f;
		original_marker.color.g = 0.7f;
		original_marker.color.b = 0.7f;
		original_marker.color.a = 0.8f;
		markers.markers.push_back(original_marker);

		// Simplified (decimated) mesh, shifted by mesh_comparison_offset so it doesn't overlap
		// the original. resolved_simplified_mesh_path_ was exported from the already-scaled
		// in-memory mesh, so no marker-level scale needed.
		visualization_msgs::msg::Marker simplified_marker;
		simplified_marker.header.frame_id = "world";
		simplified_marker.header.stamp = stamp;
		simplified_marker.ns = "simplified_mesh";
		simplified_marker.id = id++;
		simplified_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
		simplified_marker.action = visualization_msgs::msg::Marker::ADD;
		simplified_marker.mesh_resource = "file://" + resolved_simplified_mesh_path_;
		simplified_marker.mesh_use_embedded_materials = false;
		simplified_marker.pose.position = ToPoint(object_translation_world + comparison_offset);
		simplified_marker.pose.orientation.x = object_quat.x();
		simplified_marker.pose.orientation.y = object_quat.y();
		simplified_marker.pose.orientation.z = object_quat.z();
		simplified_marker.pose.orientation.w = object_quat.w();
		simplified_marker.scale.x = simplified_marker.scale.y = simplified_marker.scale.z = 1.0;
		simplified_marker.color.r = 0.2f;
		simplified_marker.color.g = 0.8f;
		simplified_marker.color.b = 0.2f;
		simplified_marker.color.a = 0.8f;
		markers.markers.push_back(simplified_marker);

		return markers;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ViewpointPlannerNode>();
	node->init();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
