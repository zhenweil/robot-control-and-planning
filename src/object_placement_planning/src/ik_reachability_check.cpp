// One-off check: is a missed viewpoint reachable at a given object position? A heavy IK search (many random starts,
// no clearance) per case, so a failure can be told apart: IK search vs truly out of reach.

#include <algorithm>
#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "object_placement_planning/closest_ik.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/scene_geometry.hpp"
#include "object_placement_planning/viewpoint_io.hpp"

namespace
{

using Joints = std::vector<double>;

// The surface the robot stands on, as in object_placement and sgd_placement.
void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
	const double size = 4.0, thickness = 0.05;	// m
	planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
	moveit_msgs::msg::CollisionObject surface;
	surface.header.frame_id = scene->getRobotModel()->getRootLinkName();
	surface.id = "mounting_surface";
	shape_msgs::msg::SolidPrimitive box;
	box.type = box.BOX;
	box.dimensions = {size, size, thickness};
	geometry_msgs::msg::Pose pose;
	pose.position.z = -0.5 * thickness;
	pose.orientation.w = 1.0;
	surface.primitives.push_back(box);
	surface.primitive_poses.push_back(pose);
	surface.operation = surface.ADD;
	scene->processCollisionObjectMsg(surface);
	for (const moveit::core::LinkModel* link : scene->getRobotModel()->getLinkModels())
		if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link) == scene->getRobotModel()->getRootLink())
			scene->getAllowedCollisionMatrixNonConst().setEntry(surface.id, link->getName(), true);
}

// What the search found for one case.
struct CheckResult
{
	int num_exact = 0;					   // random starts whose exact IK succeeded
	double best_gap_wide = std::numeric_limits<double>::max();   // closest IK, clearance closest_ik_margin
	double best_gap_narrow = std::numeric_limits<double>::max(); // closest IK, clearance narrow_margin
	int num_finished = 0;				   // closest poses (either clearance) that exact IK from them closed
	Joints best_narrow_pose;			   // the closest pose behind best_gap_narrow
};

}  // namespace

class IkReachabilityCheckNode : public rclcpp::Node
{
public:
	IkReachabilityCheckNode()
		: Node("ik_reachability_check", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
	}

	void run()
	{
		const std::string group_name = this->GetParameter<std::string>("group_name", "panda_arm");
		const std::string mesh_path = this->GetParameter<std::string>("mesh_path", "");
		const double mesh_scale = this->GetParameter<double>("mesh_scale", 0.01);
		const std::string tour_input_dir = this->GetParameter<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		const Eigen::Vector3d nominal = ToVector3(this->GetParameter<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		const Eigen::Matrix3d rotation = RotationFromRpyDeg(this->GetParameter<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		// Cases as x, y, z (abs, m), viewpoint index; repeated.
		const std::vector<double> cases = this->GetParameter<std::vector<double>>("check_cases", std::vector<double>{});
		const int num_exact_starts = static_cast<int>(this->GetParameter<int64_t>("check_exact_starts", 2000));
		const int num_closest_starts = static_cast<int>(this->GetParameter<int64_t>("check_closest_starts", 200));
		const double narrow_margin = this->GetParameter<double>("check_narrow_margin", 0.005);	// m
		closest_ik::Options options;
		options.iters = static_cast<int>(this->GetParameter<int64_t>("closest_ik_iters", 60));
		options.margin = this->GetParameter<double>("closest_ik_margin", 0.02);
		options.rot_scale = this->GetParameter<double>("rot_metric_scale", 0.1);
		options.limit_sharpness = this->GetParameter<double>("manipulability_limit_sharpness", 20.0);
		options.ignore_roll = this->GetParameter<bool>("ignore_camera_roll", false);
		options.exact_margin = this->GetParameter<double>("exact_ik_margin", 0.0);

		std::vector<Eigen::Isometry3d> viewpoints = LoadTourTcpPoses(tour_input_dir);
		for (Eigen::Isometry3d& viewpoint : viewpoints)
			viewpoint.translation() -= nominal;
		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		const moveit::core::RobotModelConstPtr model = loader.getModel();
		moveit::core::RobotState state(model);
		state.setToDefaultValues();
		const moveit::core::JointModelGroup* group = state.getJointModelGroup(group_name);
		const moveit::core::LinkModel* tool = model->getLinkModel("tool0");
		const planning_scene_monitor::PlanningSceneMonitorPtr scene = BuildLocalCollisionScene(this->shared_from_this(), mesh_path, mesh_scale, nominal, rotation);
		AddMountingSurface(scene);
		RCLCPP_INFO(this->get_logger(), "%zu cases, %d exact-IK starts and %d closest-IK starts each; clearance %.3f / %.3f m; camera roll %s", cases.size() / 4, num_exact_starts, num_closest_starts, options.margin, narrow_margin, options.ignore_roll ? "free" : "fixed");

		closest_ik::Options exact_options = options;
		exact_options.exact_only = true;
		closest_ik::Options wide_options = options;
		wide_options.try_exact_first = false;
		closest_ik::Options narrow_options = wide_options;
		narrow_options.margin = narrow_margin;
		const unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());

		for (size_t c = 0; c + 3 < cases.size(); c += 4)
		{
			const Eigen::Vector3d position(cases[c], cases[c + 1], cases[c + 2]);
			const int viewpoint_index = static_cast<int>(cases[c + 3]);
			scene_geometry::MoveObject(scene, scene_geometry::MakeIsometry(position, rotation));
			const Eigen::Isometry3d target = scene_geometry::Translation(position) * viewpoints[static_cast<size_t>(viewpoint_index)];
			CheckResult result;
			std::mutex result_mutex;
			std::vector<std::thread> threads;
			for (unsigned int t = 0; t < num_threads; ++t)
				threads.emplace_back([&, t]()
				{
					moveit::core::RobotState thread_state(state);
					random_numbers::RandomNumberGenerator rng(static_cast<uint32_t>(1000 * c + t + 1));
					CheckResult local;
					Joints start;
					for (int i = static_cast<int>(t); i < num_exact_starts; i += static_cast<int>(num_threads))
					{
						thread_state.setToRandomPositions(group, rng);
						thread_state.copyJointGroupPositions(group, start);
						local.num_exact += closest_ik::Solve(scene, thread_state, group, tool, target, start, exact_options).exact ? 1 : 0;
					}
					for (int i = static_cast<int>(t); i < num_closest_starts; i += static_cast<int>(num_threads))
					{
						thread_state.setToRandomPositions(group, rng);
						thread_state.copyJointGroupPositions(group, start);
						for (const bool narrow : {false, true})
						{
							const closest_ik::Result closest = closest_ik::Solve(scene, thread_state, group, tool, target, start, narrow ? narrow_options : wide_options);
							if (!closest.found)
								continue;
							double& best = narrow ? local.best_gap_narrow : local.best_gap_wide;
							if (narrow && closest.gap < best)
								local.best_narrow_pose = closest.q;
							best = std::min(best, closest.exact ? 0.0 : closest.gap);
							local.num_finished += closest_ik::Solve(scene, thread_state, group, tool, target, closest.q, exact_options).exact ? 1 : 0;
						}
					}
					std::lock_guard<std::mutex> lock(result_mutex);
					result.num_exact += local.num_exact;
					result.best_gap_wide = std::min(result.best_gap_wide, local.best_gap_wide);
					if (local.best_gap_narrow < result.best_gap_narrow)
						result.best_narrow_pose = local.best_narrow_pose;
					result.best_gap_narrow = std::min(result.best_gap_narrow, local.best_gap_narrow);
					result.num_finished += local.num_finished;
				});
			for (std::thread& thread : threads)
				thread.join();
			// What the closest pose touches: pairs within 1.5 cm, nearest first.
			std::string blockers;
			if (!result.best_narrow_pose.empty())
			{
				moveit::core::RobotState pose_state(state);
				pose_state.setJointGroupPositions(group, result.best_narrow_pose);
				pose_state.update();
				const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(scene, pose_state, group, closest_ik::CloseRequest(pose_state, group, 0.015), 0.015);
				std::vector<size_t> order(pairs.dists.size());
				for (size_t k = 0; k < order.size(); ++k)
					order[k] = k;
				std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return pairs.dists[a] < pairs.dists[b]; });
				for (size_t k = 0; k < std::min<size_t>(order.size(), 4); ++k)
					blockers += " [" + pairs.names[order[k]] + " " + std::to_string(pairs.dists[order[k]] * 1000.0).substr(0, 4) + " mm]";
			}
			RCLCPP_INFO(this->get_logger(), "object abs (%.4f, %.4f, %.4f) m, viewpoint %d: exact IK %d/%d random starts; closest IK best gap %.4f m (clearance %.3f), %.4f m (clearance %.3f); exact IK from a closest pose closed it %d/%d times -> %s; closest pose (clearance %.3f) is near:%s", position.x(), position.y(), position.z(), viewpoint_index, result.num_exact, num_exact_starts, result.best_gap_wide, options.margin, result.best_gap_narrow, narrow_margin, result.num_finished, 2 * num_closest_starts, (result.num_exact > 0 || result.num_finished > 0) ? "REACHABLE" : "not found", narrow_margin, blockers.empty() ? " nothing within 15 mm" : blockers.c_str());
		}
		RCLCPP_INFO(this->get_logger(), "Done.");
	}

private:
	template <typename T>
	T GetParameter(const std::string& name, const T& fallback)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, fallback);
		return this->get_parameter(name).get_value<T>();
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<IkReachabilityCheckNode>();
	std::thread spin([node]() { rclcpp::spin(node); });
	node->run();
	rclcpp::shutdown();
	spin.join();
	return 0;
}
