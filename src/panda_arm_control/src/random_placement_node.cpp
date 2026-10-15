// Random-sampling baseline for object placement: draws object positions uniformly in a box and keeps the one that
// reaches the most tour viewpoints (a viewpoint counts as reached if it has a collision-free IK solution). Stops at
// the first position that reaches every viewpoint, or when the time / sample budget runs out. Independent of the
// base_gradient method: own scene setup, own IK check.

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "panda_arm_control/real_cost_planning.hpp"
#include "panda_arm_control/viewpoint_io.hpp"

namespace
{

geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Isometry3d& t)
{
	geometry_msgs::msg::Pose p;
	p.position.x = t.translation().x();
	p.position.y = t.translation().y();
	p.position.z = t.translation().z();
	const Eigen::Quaterniond q(t.rotation());
	p.orientation.x = q.x();
	p.orientation.y = q.y();
	p.orientation.z = q.z();
	p.orientation.w = q.w();
	return p;
}

// The surface the robot stands on (top face at the base plane), as in base_gradient, so both see the same world.
void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
	const double size = 4.0, thickness = 0.05;  // m
	planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
	const std::string base_link = scene->getRobotModel()->getRootLinkName();
	moveit_msgs::msg::CollisionObject surface;
	surface.header.frame_id = base_link;
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
	scene->getAllowedCollisionMatrixNonConst().setEntry(surface.id, base_link, true);
}

void MoveObject(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& frame,
	const Eigen::Isometry3d& pose)
{
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = frame;
	obj.id = "object";
	obj.pose = ToPoseMsg(pose);
	obj.operation = obj.MOVE;
	planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
	scene->processCollisionObjectMsg(obj);
}

}  // namespace

class RandomPlacementNode : public rclcpp::Node
{
public:
	RandomPlacementNode()
		: Node("random_placement", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
	}

	void run()
	{
		const std::string group_name = param<std::string>("group_name", "panda_arm");
		std::string mesh_path = param<std::string>("mesh_path", "");
		const double mesh_scale = param<double>("mesh_scale", 0.01);
		const std::string tour_input_dir = param<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		const std::string output_dir = param<std::string>("output_dir", "/tmp/random_placement_output");
		const std::vector<double> home = param<std::vector<double>>(
			"initial_joints", std::vector<double>{0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785});
		const Eigen::Vector3d nominal =
			ToVector3(param<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		const Eigen::Matrix3d rotation =
			RotationFromRpyDeg(param<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		// Same bounds and IK budget names as base_gradient's YAML, so one params file serves both.
		const Eigen::Vector3d lo(param<double>("bg_x_min", -0.6), param<double>("bg_y_min", -0.6), param<double>("bg_z_min", -0.2));
		const Eigen::Vector3d hi(param<double>("bg_x_max", 0.6), param<double>("bg_y_max", 0.6), param<double>("bg_z_max", 0.2));
		const int ik_attempts = std::max(1, static_cast<int>(param<int64_t>("ik_attempts", 30)));
		const int seed = static_cast<int>(param<int64_t>("random_seed", 42));
		const double time_budget = param<double>("rp_time_budget_sec", 90.0);
		const int max_samples = static_cast<int>(param<int64_t>("rp_max_samples", 0));  // 0 = no limit

		if (mesh_path.empty())
			mesh_path = ament_index_cpp::get_package_share_directory("panda_arm_control") + "/meshes/bunny_holding_eggs.stl";
		const std::vector<Eigen::Isometry3d> tour = LoadTourTcpPoses(tour_input_dir);
		const int n = static_cast<int>(tour.size());
		RCLCPP_INFO(get_logger(), "Loaded %d tour poses; sampling in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s",
			n, lo.x(), hi.x(), lo.y(), hi.y(), lo.z(), hi.z(), time_budget);

		robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
		const moveit::core::RobotModelConstPtr model = loader.getModel();
		moveit::core::RobotState state(model);
		state.setToDefaultValues();
		const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
		auto psm = BuildLocalCollisionScene(shared_from_this(), mesh_path, mesh_scale, nominal, rotation);
		AddMountingSurface(psm);
		const std::string frame = model->getRootLinkName();

		auto valid = [&](moveit::core::RobotState* s, const moveit::core::JointModelGroup* g, const double* q) {
			s->setJointGroupPositions(g, q);
			s->update();
			planning_scene_monitor::LockedPlanningSceneRO scene(psm);
			return scene->isStateValid(*s, g->getName());
		};
		random_numbers::RandomNumberGenerator ik_rng(static_cast<boost::uint32_t>(std::max(1, seed)));
		// Viewpoints with a collision-free IK solution: the home pose first, then random starts, ik_attempts in all.
		auto count_reached = [&](const Eigen::Vector3d& at) {
			const Eigen::Isometry3d shift(Eigen::Translation3d(at - nominal));
			Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
			object_pose.translation() = at;
			object_pose.linear() = rotation;
			MoveObject(psm, frame, object_pose);
			int reached = 0;
			for (const Eigen::Isometry3d& tcp : tour)
			{
				const geometry_msgs::msg::Pose target = ToPoseMsg(shift * tcp);
				state.setJointGroupPositions(jmg, home);
				for (int a = 0; a < ik_attempts; ++a)
				{
					if (a > 0)
						state.setToRandomPositions(jmg, ik_rng);
					if (state.setFromIK(jmg, target, "tool0", 1e-9, valid))
					{
						++reached;
						break;
					}
				}
			}
			return reached;
		};

		std::mt19937 rng(static_cast<unsigned int>(seed));
		std::uniform_real_distribution<double> ux(lo.x(), hi.x()), uy(lo.y(), hi.y()), uz(lo.z(), hi.z());
		const auto t0 = std::chrono::steady_clock::now();
		auto elapsed = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };
		Eigen::Vector3d best = nominal;
		int best_reached = -1, samples = 0;
		while (rclcpp::ok() && elapsed() < time_budget && (max_samples <= 0 || samples < max_samples))
		{
			const Eigen::Vector3d at(ux(rng), uy(rng), uz(rng));
			const int reached = count_reached(at);
			++samples;
			if (reached > best_reached)
			{
				best_reached = reached;
				best = at;
				RCLCPP_INFO(get_logger(), "sample %d (%.1f s): abs (%.4f, %.4f, %.4f) m reaches %d/%d  <-- best",
					samples, elapsed(), at.x(), at.y(), at.z(), reached, n);
			}
			if (best_reached == n)
				break;
		}

		const double secs = elapsed();
		if (best_reached == n)
			RCLCPP_INFO(get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses (%d samples, %.1f s).",
				best.x(), best.y(), best.z(), n, samples, secs);
		else
			RCLCPP_WARN(get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses (%d samples, %.1f s).",
				best.x(), best.y(), best.z(), std::max(0, best_reached), n, samples, secs);

		std::filesystem::create_directories(output_dir);
		Json::Value out;
		out["ok"] = best_reached == n;
		out["num_reachable"] = best_reached;
		out["num_total"] = n;
		out["x"] = best.x();
		out["y"] = best.y();
		out["z"] = best.z();
		out["samples"] = samples;
		out["seconds"] = secs;
		std::ofstream(output_dir + "/random_placement_result.json") << out;
	}

private:
	template <typename T>
	T param(const std::string& name, const T& fallback)
	{
		if (!has_parameter(name))
			declare_parameter(name, fallback);
		return get_parameter(name).get_value<T>();
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RandomPlacementNode>();
	std::thread spin([node]() { rclcpp::spin(node); });
	node->run();
	rclcpp::shutdown();
	spin.join();
	return 0;
}
