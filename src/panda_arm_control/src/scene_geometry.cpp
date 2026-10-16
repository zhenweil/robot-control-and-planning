#include "panda_arm_control/scene_geometry.hpp"

#include <cmath>

#include <moveit_msgs/msg/collision_object.hpp>

namespace scene_geometry
{

Eigen::Isometry3d Translation(const Eigen::Vector3d& p)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = p;
	return t;
}

Eigen::Isometry3d MakeIsometry(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
{
	Eigen::Isometry3d t = Translation(translation);
	t.linear() = rotation;
	return t;
}

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

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& v)
{
	geometry_msgs::msg::Point p;
	p.x = v.x();
	p.y = v.y();
	p.z = v.z();
	return p;
}

bool IsStateValid(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState* state,
	const moveit::core::JointModelGroup* group, const double* q)
{
	state->setJointGroupPositions(group, q);
	state->update();
	planning_scene_monitor::LockedPlanningSceneRO locked(scene);
	return locked->isStateValid(*state, group->getName());
}

void MoveObject(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const Eigen::Isometry3d& pose)
{
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = "panda_link0";
	obj.id = "object";
	obj.pose = ToPoseMsg(pose);
	obj.operation = obj.MOVE;
	planning_scene_monitor::LockedPlanningSceneRW locked(scene);
	locked->processCollisionObjectMsg(obj);
}

Vector6d PoseError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& tool)
{
	const Eigen::AngleAxisd rot(target.linear() * tool.linear().transpose());
	Vector6d e;
	e.head<3>() = target.translation() - tool.translation();
	e.tail<3>() = rot.axis() * rot.angle();
	return e;
}

double PoseGap(const Vector6d& e, double rot_scale)
{
	return std::sqrt(e.head<3>().squaredNorm() + rot_scale * rot_scale * e.tail<3>().squaredNorm());
}

Eigen::MatrixXd DampedPinv(const Eigen::MatrixXd& J, double damping)
{
	Eigen::MatrixXd JJt = J * J.transpose();
	JJt.diagonal().array() += damping * damping;
	return J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(J.rows(), J.rows()));
}

}  // namespace scene_geometry
