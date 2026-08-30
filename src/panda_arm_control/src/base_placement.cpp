#include "panda_arm_control/base_placement.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <thread>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <json/json.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>

namespace
{

// Eigen::Translation3d * Matrix3d yields an Affine3d (not Isometry3d, even for a rotation
// matrix), so build the Isometry directly instead of relying on operator* here.
Eigen::Isometry3d MakeIsometry(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = translation;
	t.linear() = rotation;
	return t;
}

// Pure translation offset applied to the object (and its viewpoints) in the base frame. Rotation
// is held at 0 in this baseline (see base_placement.hpp).
Eigen::Isometry3d MakeObjectOffsetXform(double x, double y, double z)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = Eigen::Vector3d(x, y, z);
	return t;
}

geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Isometry3d& t)
{
	geometry_msgs::msg::Pose p;
	p.position.x = t.translation().x();
	p.position.y = t.translation().y();
	p.position.z = t.translation().z();
	Eigen::Quaterniond q(t.rotation());
	p.orientation.x = q.x();
	p.orientation.y = q.y();
	p.orientation.z = q.z();
	p.orientation.w = q.w();
	return p;
}

// Mirrors real_cost_planning.cpp's private helper of the same name/signature.
bool IsStateCollisionFree(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, moveit::core::RobotState* state,
	const moveit::core::JointModelGroup* group, const double* joint_positions)
{
	state->setJointGroupPositions(group, joint_positions);
	state->update();
	planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
	return locked_scene->isStateValid(*state, group->getName());
}

// MOVE only updates the registered object's pose -- cheap, no mesh reload.
void SetObjectPose(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const Eigen::Isometry3d& pose)
{
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = "panda_link0";
	obj.id = "object";
	obj.pose = ToPoseMsg(pose);
	obj.operation = obj.MOVE;

	planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor);
	locked_scene->processCollisionObjectMsg(obj);
}

struct Offset3
{
	double x = 0.0, y = 0.0, z = 0.0;
};

double RandomUniform(std::mt19937& rng, double lo, double hi)
{
	std::uniform_real_distribution<double> dist(lo, hi);
	return dist(rng);
}

Offset3 ClampToBounds(Offset3 o, const BasePlacementBounds& b)
{
	o.x = std::clamp(o.x, b.x_min, b.x_max);
	o.y = std::clamp(o.y, b.y_min, b.y_max);
	o.z = std::clamp(o.z, b.z_min, b.z_max);
	return o;
}

Offset3 RandomInBounds(const BasePlacementBounds& b, std::mt19937& rng)
{
	return {RandomUniform(rng, b.x_min, b.x_max), RandomUniform(rng, b.y_min, b.y_max),
			RandomUniform(rng, b.z_min, b.z_max)};
}

Offset3 Mean3(const std::vector<Offset3>& offsets, const std::vector<bool>& placed)
{
	double sx = 0.0, sy = 0.0, sz = 0.0;
	int n = 0;
	for (size_t i = 0; i < offsets.size(); ++i)
	{
		if (!placed[i])
			continue;
		sx += offsets[i].x;
		sy += offsets[i].y;
		sz += offsets[i].z;
		++n;
	}
	if (n == 0)
		return {};
	return {sx / n, sy / n, sz / n};
}

double JointL1Distance(const std::vector<double>& a, const std::vector<double>& b)
{
	double d = 0.0;
	for (size_t k = 0; k < a.size() && k < b.size(); ++k)
		d += std::abs(a[k] - b[k]);
	return d;
}

// Step-1 relaxation: for one viewpoint, find ANY collision-free object offset that reaches it
// (mobile-object relaxation), seeded from the previous point's joints then a few random restarts.
bool TryReachPointRelaxed(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const Eigen::Isometry3d& target_pose_original, const Offset3& off,
	const std::vector<double>& seed_joints, double ik_timeout, int ik_retries, std::vector<double>* out_joints)
{
	const Eigen::Isometry3d xform = MakeObjectOffsetXform(off.x, off.y, off.z);
	SetObjectPose(planning_scene_monitor, xform * object_pose_original);
	const geometry_msgs::msg::Pose target_local = ToPoseMsg(xform * target_pose_original);

	auto validity_callback = [&planning_scene_monitor](
								 moveit::core::RobotState* s, const moveit::core::JointModelGroup* g,
								 const double* jp) { return IsStateCollisionFree(planning_scene_monitor, s, g, jp); };

	state.setJointGroupPositions(jmg, seed_joints);
	bool ok = state.setFromIK(jmg, target_local, "tool0", ik_timeout, validity_callback);
	for (int attempt = 0; !ok && attempt < ik_retries; ++attempt)
	{
		state.setToRandomPositions(jmg);
		ok = state.setFromIK(jmg, target_local, "tool0", ik_timeout, validity_callback);
	}
	if (ok)
		state.copyJointGroupPositions(jmg, *out_joints);
	return ok;
}

void PublishRelaxationProgress(
	const rclcpp::Node::SharedPtr& node, const BasePlacementParams& params, const std::vector<Offset3>& offsets,
	const std::vector<bool>& placed, const Offset3& mean)
{
	if (!params.progress_pub)
		return;

	const rclcpp::Time stamp = node->now();
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;
	for (size_t i = 0; i < offsets.size(); ++i)
	{
		visualization_msgs::msg::Marker dot;
		dot.header.frame_id = "world";
		dot.header.stamp = stamp;
		dot.ns = "base_placement_relaxation";
		dot.id = id++;
		dot.type = visualization_msgs::msg::Marker::SPHERE;
		dot.action = visualization_msgs::msg::Marker::ADD;
		dot.pose.position.x = offsets[i].x;
		dot.pose.position.y = offsets[i].y;
		dot.pose.position.z = offsets[i].z;
		dot.pose.orientation.w = 1.0;
		dot.scale.x = dot.scale.y = dot.scale.z = 0.015;
		const bool p = i < placed.size() && placed[i];
		dot.color.r = p ? 0.1f : 0.8f;
		dot.color.g = p ? 0.6f : 0.2f;
		dot.color.b = 0.9f;
		dot.color.a = 0.9f;
		markers.markers.push_back(dot);
	}
	visualization_msgs::msg::Marker mean_marker;
	mean_marker.header.frame_id = "world";
	mean_marker.header.stamp = stamp;
	mean_marker.ns = "base_placement_relaxation_mean";
	mean_marker.id = id++;
	mean_marker.type = visualization_msgs::msg::Marker::SPHERE;
	mean_marker.action = visualization_msgs::msg::Marker::ADD;
	mean_marker.pose.position.x = mean.x;
	mean_marker.pose.position.y = mean.y;
	mean_marker.pose.position.z = mean.z;
	mean_marker.pose.orientation.w = 1.0;
	mean_marker.scale.x = mean_marker.scale.y = mean_marker.scale.z = 0.05;
	mean_marker.color.r = 1.0f;
	mean_marker.color.g = 0.85f;
	mean_marker.color.a = 1.0f;
	markers.markers.push_back(mean_marker);

	params.progress_pub->publish(markers);
	if (params.visualize_progress_delay_sec > 0.0)
		std::this_thread::sleep_for(std::chrono::duration<double>(params.visualize_progress_delay_sec));
}

struct RestartResult
{
	Offset3 offset;
	ObjectOffsetScore score;
};

// One B*-style restart: (optionally) a mobile-object relaxation for the starting point, then an
// axis-aligned pattern search that minimizes ScoreObjectOffset -- probe +/- step on x, y, z, step
// to the best improving neighbour, shrink the step when none improves, stop at min_step.
RestartResult RunOneRestart(
	const rclcpp::Node::SharedPtr& node, int restart_number, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& start_reference_joints,
	const BasePlacementParams& params, const BaseGradientParams& score_params, const std::vector<int>& fixed_order,
	std::mt19937& rng, bool relax_start)
{
	const int n = static_cast<int>(tour_tcp_poses_original.size());
	const Eigen::Isometry3d object_pose_original =
		MakeIsometry(object_translation_original, object_rotation_original);

	auto score = [&](const Offset3& o) {
		return ScoreObjectOffset(
			node, robot_model, planning_scene_monitor, group_name, object_translation_original,
			object_rotation_original, tour_tcp_poses_original, start_reference_joints, score_params,
			std::array<double, 5>{o.x, o.y, o.z, 0.0, 0.0}, fixed_order);
	};

	Offset3 start;
	if (relax_start)
	{
		moveit::core::RobotState state(robot_model);
		state.setToDefaultValues();
		const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
		std::vector<Offset3> per_point(n);
		std::vector<bool> placed(n, false);
		std::vector<double> seed = start_reference_joints;
		for (int i = 0; i < n; ++i)
		{
			for (int attempt = 0; attempt < std::max(1, params.candidates_per_point); ++attempt)
			{
				const Offset3 cand = RandomInBounds(params.bounds, rng);
				std::vector<double> sol;
				if (TryReachPointRelaxed(
						state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original[i], cand,
						seed, params.ik_timeout, params.ik_retries_per_point, &sol))
				{
					per_point[i] = cand;
					placed[i] = true;
					seed = sol;
					break;
				}
			}
		}
		const int num_placed = std::count(placed.begin(), placed.end(), true);
		start = Mean3(per_point, placed);
		PublishRelaxationProgress(node, params, per_point, placed, start);
		RCLCPP_INFO(
			node->get_logger(),
			"[restart %d] relaxation: %d/%d points found a feasible offset; mean start (%.3f, %.3f, %.3f)",
			restart_number, num_placed, n, start.x, start.y, start.z);
	}
	else
	{
		start = RandomInBounds(params.bounds, rng);
		RCLCPP_INFO(
			node->get_logger(), "[restart %d] random start (%.3f, %.3f, %.3f)", restart_number, start.x, start.y,
			start.z);
	}
	start = ClampToBounds(start, params.bounds);

	Offset3 cur = start;
	ObjectOffsetScore cur_score = score(cur);
	double step = params.initial_step;
	RCLCPP_INFO(
		node->get_logger(), "[restart %d] iter 0: offset (%.3f, %.3f, %.3f)  honest %.3f  reach %d/%d",
		restart_number, cur.x, cur.y, cur.z, cur_score.honest_cost, cur_score.num_reachable, n);

	for (int iter = 0; iter < params.max_outer_iterations && rclcpp::ok(); ++iter)
	{
		Offset3 best_nb = cur;
		ObjectOffsetScore best_nb_score = cur_score;
		for (int axis = 0; axis < 3; ++axis)
		{
			for (int dir = -1; dir <= 1; dir += 2)
			{
				Offset3 nb = cur;
				double* comp = axis == 0 ? &nb.x : axis == 1 ? &nb.y : &nb.z;
				*comp += dir * step;
				nb = ClampToBounds(nb, params.bounds);
				if (std::abs(nb.x - cur.x) < 1e-9 && std::abs(nb.y - cur.y) < 1e-9 && std::abs(nb.z - cur.z) < 1e-9)
					continue;
				const ObjectOffsetScore s = score(nb);
				if (s.weighted_cost < best_nb_score.weighted_cost)
				{
					best_nb = nb;
					best_nb_score = s;
				}
			}
		}

		const double gain = cur_score.weighted_cost - best_nb_score.weighted_cost;
		if (gain > params.convergence_tolerance_cost)
		{
			cur = best_nb;
			cur_score = best_nb_score;
		}
		else
		{
			step *= params.step_shrink;
		}
		RCLCPP_INFO(
			node->get_logger(),
			"[restart %d] iter %d/%d: offset (%.3f, %.3f, %.3f)  honest %.3f  reach %d/%d  step %.4f",
			restart_number, iter + 1, params.max_outer_iterations, cur.x, cur.y, cur.z, cur_score.honest_cost,
			cur_score.num_reachable, n, step);
		if (step < params.min_step)
			break;
	}

	return {cur, cur_score};
}

}  // namespace

BasePlacementResult SolveBasePlacement(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& start_reference_joints,
	const BasePlacementParams& params, const BaseGradientParams& score_params)
{
	const Eigen::Isometry3d object_pose_original =
		MakeIsometry(object_translation_original, object_rotation_original);
	const int n = static_cast<int>(tour_tcp_poses_original.size());

	// B* takes the visiting order as a fixed input. tour_tcp_poses_original is already in that
	// order (a viewpoint_planner_* run wrote selected_robot_poses.json in visiting sequence), so
	// the frozen order is simply 0, 1, ..., n-1. Only the object placement is optimized; each
	// score picks the cheapest IK branch per stop along this order (SolveFixedOrder).
	std::vector<int> fixed_order(n);
	for (int i = 0; i < n; ++i)
		fixed_order[i] = i;

	const ObjectOffsetScore nominal = ScoreObjectOffset(
		node, robot_model, planning_scene_monitor, group_name, object_translation_original, object_rotation_original,
		tour_tcp_poses_original, start_reference_joints, score_params, std::array<double, 5>{0.0, 0.0, 0.0, 0.0, 0.0},
		fixed_order);
	RCLCPP_INFO(
		node->get_logger(),
		"B* baseline: input tour order (%d poses) held fixed; nominal-offset honest cost %.3f, reach %d/%d. "
		"Optimizing object placement only.",
		n, nominal.honest_cost, nominal.num_reachable, n);

	RestartResult best;
	bool have_best = false;
	for (int r = 0; r < std::max(1, params.num_restarts) && rclcpp::ok(); ++r)
	{
		std::mt19937 rng(static_cast<unsigned int>(params.random_seed + r));
		RestartResult c = RunOneRestart(
			node, r + 1, robot_model, planning_scene_monitor, group_name, object_translation_original,
			object_rotation_original, tour_tcp_poses_original, start_reference_joints, params, score_params, fixed_order,
			rng, /*relax_start=*/r == 0);
		RCLCPP_INFO(
			node->get_logger(),
			"[restart %d/%d] done: offset (%.4f, %.4f, %.4f) m  honest %.3f  reach %d/%d%s",
			r + 1, params.num_restarts, c.offset.x, c.offset.y, c.offset.z, c.score.honest_cost, c.score.num_reachable,
			n, (have_best && c.score.weighted_cost < best.score.weighted_cost) ? "  <-- new best" : "");
		if (!have_best || c.score.weighted_cost < best.score.weighted_cost)
		{
			best = c;
			have_best = true;
		}
	}

	SetObjectPose(planning_scene_monitor, object_pose_original);  // leave the scene as we found it

	BasePlacementResult result;
	result.num_total = n;
	if (have_best)
	{
		result.x = best.offset.x;
		result.y = best.offset.y;
		result.z = best.offset.z;
		result.tour_order = best.score.tour;
		result.joint_solutions = best.score.joints;
		result.num_reachable = best.score.num_reachable;
		result.ok = best.score.all_reachable;
		result.weighted_cost = best.score.weighted_cost;
		result.honest_cost = best.score.honest_cost;
		const std::vector<double>* prev = &start_reference_joints;
		for (const auto& q : best.score.joints)
		{
			if (!q.empty())
			{
				result.total_joint_path_length += JointL1Distance(*prev, q);
				prev = &q;
			}
		}
	}

	if (result.ok)
		RCLCPP_INFO(
			node->get_logger(),
			"Done. Object offset (%.4f, %.4f, %.4f) m -- reaches all %d viewpoints, honest cost %.3f (joint L1 path "
			"%.3f).",
			result.x, result.y, result.z, n, result.honest_cost, result.total_joint_path_length);
	else
		RCLCPP_WARN(
			node->get_logger(),
			"Done. Object offset (%.4f, %.4f, %.4f) m -- only reaches %d/%d viewpoints.",
			result.x, result.y, result.z, result.num_reachable, n);

	return result;
}

void ApplyBasePlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original, double x,
	double y, double z)
{
	const Eigen::Isometry3d object_pose_original =
		MakeIsometry(object_translation_original, object_rotation_original);
	SetObjectPose(planning_scene_monitor, MakeObjectOffsetXform(x, y, z) * object_pose_original);
}

void ExportBasePlacementResult(const std::string& output_dir, const BasePlacementResult& result)
{
	std::filesystem::create_directories(output_dir);

	Json::Value root;
	root["ok"] = result.ok;
	root["num_reachable"] = result.num_reachable;
	root["num_total"] = result.num_total;
	root["x"] = result.x;
	root["y"] = result.y;
	root["z"] = result.z;
	root["weighted_cost"] = result.weighted_cost;
	root["honest_cost"] = result.honest_cost;
	root["total_joint_path_length"] = result.total_joint_path_length;

	Json::Value tour(Json::arrayValue);
	for (int idx : result.tour_order)
		tour.append(idx);
	root["tour_order"] = tour;

	Json::Value joints(Json::arrayValue);
	for (const auto& sol : result.joint_solutions)
	{
		Json::Value entry(Json::arrayValue);
		for (double v : sol)
			entry.append(v);
		joints.append(entry);
	}
	root["joint_solutions"] = joints;

	const std::string json_path = output_dir + "/base_placement_result.json";
	std::ofstream json_file(json_path);
	Json::StreamWriterBuilder writer_builder;
	writer_builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
	writer->write(root, &json_file);

	printf("Saved base placement result JSON: %s\n", json_path.c_str());
}

visualization_msgs::msg::MarkerArray BuildBasePlacementMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const BasePlacementResult& result)
{
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;

	const Eigen::Isometry3d xform = MakeObjectOffsetXform(result.x, result.y, result.z);
	const Eigen::Isometry3d object_pose_original =
		MakeIsometry(object_translation_original, object_rotation_original);

	visualization_msgs::msg::Marker mesh_marker;
	mesh_marker.header.frame_id = "world";
	mesh_marker.header.stamp = stamp;
	mesh_marker.ns = "recommended_offset_object";
	mesh_marker.id = id++;
	mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	mesh_marker.action = visualization_msgs::msg::Marker::ADD;
	mesh_marker.mesh_resource = "file://" + resolved_mesh_path;
	mesh_marker.mesh_use_embedded_materials = false;
	mesh_marker.pose = ToPoseMsg(xform * object_pose_original);
	mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = mesh_scale;
	mesh_marker.color.r = 0.7f;
	mesh_marker.color.g = 0.7f;
	mesh_marker.color.b = 0.7f;
	mesh_marker.color.a = 0.5f;
	markers.markers.push_back(mesh_marker);

	std::vector<char> reachable(tour_tcp_poses_original.size(), 0);
	for (int idx : result.tour_order)
		if (idx >= 0 && idx < static_cast<int>(reachable.size()))
			reachable[idx] = 1;

	visualization_msgs::msg::Marker line;
	line.header.frame_id = "world";
	line.header.stamp = stamp;
	line.ns = "recommended_offset_tour";
	line.id = id++;
	line.type = visualization_msgs::msg::Marker::LINE_STRIP;
	line.action = visualization_msgs::msg::Marker::ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x = 0.002;
	line.color.r = 0.1f;
	line.color.g = 0.9f;
	line.color.b = 0.1f;
	line.color.a = 0.9f;

	auto waypoint_marker = [&](int vp) {
		const Eigen::Isometry3d local = xform * tour_tcp_poses_original[vp];
		visualization_msgs::msg::Marker sphere;
		sphere.header.frame_id = "world";
		sphere.header.stamp = stamp;
		sphere.ns = "recommended_offset_waypoints";
		sphere.id = id++;
		sphere.type = visualization_msgs::msg::Marker::SPHERE;
		sphere.action = visualization_msgs::msg::Marker::ADD;
		sphere.pose.position.x = local.translation().x();
		sphere.pose.position.y = local.translation().y();
		sphere.pose.position.z = local.translation().z();
		sphere.pose.orientation.w = 1.0;
		sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.008;
		sphere.color.r = reachable[vp] ? 0.1f : 0.9f;
		sphere.color.g = reachable[vp] ? 0.9f : 0.1f;
		sphere.color.b = 0.1f;
		sphere.color.a = 1.0f;
		return sphere;
	};

	// Waypoints connected in the winning visiting order; then any unreachable ones as loose dots.
	for (int vp : result.tour_order)
	{
		if (vp < 0 || vp >= static_cast<int>(tour_tcp_poses_original.size()))
			continue;
		const visualization_msgs::msg::Marker sphere = waypoint_marker(vp);
		markers.markers.push_back(sphere);
		line.points.push_back(sphere.pose.position);
	}
	for (size_t vp = 0; vp < tour_tcp_poses_original.size(); ++vp)
		if (!reachable[vp])
			markers.markers.push_back(waypoint_marker(static_cast<int>(vp)));

	markers.markers.push_back(line);
	return markers;
}
