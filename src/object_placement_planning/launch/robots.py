"""Per-robot settings shared by the launch files: robot:=panda (default), kinova (Kinova Gen3 7-DOF), xarm7 (UFactory xArm7), ur5e or ur3e (Universal Robots, 6-DOF)."""
import xml.etree.ElementTree as ElementTree

import yaml

from moveit_configs_utils import MoveItConfigsBuilder

# home: the tour's start pose (overrides object_pose.yaml's initial_joints). The other robots' homes put tool0 at the
# Panda home's tool0 pose, so all robots start the tour from the same camera pose.
ROBOTS = {
    "panda": {
        "name": "panda",
        "package": "panda_arm_moveit",
        "urdf": "config/panda.urdf.xacro",
        "srdf": "config/panda.srdf",
        "controllers": "config/gripper_moveit_controllers.yaml",
        "group": "panda_arm",
        "base_link": "panda_link0",
        "home": {"panda_joint1": 0.0, "panda_joint2": -0.8745, "panda_joint3": 0.0, "panda_joint4": -2.356, "panda_joint5": 0.0, "panda_joint6": 1.571, "panda_joint7": 0.785},
    },
    "kinova": {
        "name": "gen3",
        "package": "kinova_arm_moveit",
        "urdf": "urdf/gen3.urdf.xacro",
        "srdf": "config/gen3.srdf",
        "controllers": "config/moveit_controllers.yaml",
        "group": "manipulator",
        "base_link": None,  # the URDF has its own world link
        "home": {"joint_1": -0.0394, "joint_2": -0.0876, "joint_3": -0.0461, "joint_4": 1.4027, "joint_5": 0.0037, "joint_6": 1.7371, "joint_7": -3.226},
    },
    "xarm7": {
        "name": "xarm7",
        "package": "xarm_arm_moveit",
        "urdf": "urdf/xarm7.urdf",
        "srdf": "config/xarm7.srdf",
        "controllers": "config/moveit_controllers.yaml",
        "group": "xarm7",
        "base_link": "link_base",
        "home": {"joint1": 0.0, "joint2": -0.5638, "joint3": 0.0, "joint4": 1.2697, "joint5": 0.0, "joint6": 1.744, "joint7": 0.0},
    },
    # 6 joints: no self-motion, so tour-placement's self-motion steps and the roomier-pose nudges do nothing.
    "ur5e": {
        "name": "ur5e",
        "package": "ur_arm_moveit",
        "urdf": "urdf/ur5e.urdf",
        "srdf": "config/ur5e.srdf",
        "controllers": "config/moveit_controllers.yaml",
        "group": "ur_manipulator",
        "base_link": "base_link",
        "home": {"shoulder_pan_joint": -0.5083, "shoulder_lift_joint": -2.0888, "elbow_joint": 1.6225, "wrist_1_joint": -1.1827, "wrist_2_joint": -1.5273, "wrist_3_joint": -2.0778},
    },
    # 500 mm reach: its home keeps the Panda home's camera orientation and x/y but sits lower (z = 0.30 m).
    "ur3e": {
        "name": "ur3e",
        "package": "ur_arm_moveit",
        "urdf": "urdf/ur3e.urdf",
        "srdf": "config/ur3e.srdf",
        "controllers": "config/moveit_controllers.yaml",
        "group": "ur_manipulator",
        "base_link": "base_link",
        "home": {"shoulder_pan_joint": -0.4889, "shoulder_lift_joint": -1.7634, "elbow_joint": 1.5398, "wrist_1_joint": -1.4262, "wrist_2_joint": -1.5288, "wrist_3_joint": -2.0584},
    },
}


def GetRobot(name):
    if name not in ROBOTS:
        raise RuntimeError(f"unknown robot '{name}'; choose one of {sorted(ROBOTS)}")
    return ROBOTS[name]


def GetMoveItConfig(robot):
    return (
        MoveItConfigsBuilder(robot["name"], package_name=robot["package"])
        .robot_description(file_path=robot["urdf"])
        .robot_description_semantic(file_path=robot["srdf"])
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path=robot["controllers"])
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


# Hard joint margin: every revolute joint's range (as MoveIt reads it from the URDF) shrunk by margin (rad) on both
# sides, as joint_limits.yaml position limits, so IK, sampling and planning all treat the rest as out of range. 0 = off.
def ShrinkJointRanges(moveit_config, margin):
    if margin <= 0.0:
        return moveit_config
    limits = moveit_config.joint_limits["robot_description_planning"]["joint_limits"]
    for joint in ElementTree.fromstring(str(moveit_config.robot_description["robot_description"])).iter("joint"):
        limit = joint.find("limit")
        if joint.get("type") != "revolute" or limit is None or joint.get("name") not in limits:
            continue
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        safety = joint.find("safety_controller")  # MoveIt uses the tighter soft limits when given
        if safety is not None:
            lower = max(lower, float(safety.get("soft_lower_limit", lower)))
            upper = min(upper, float(safety.get("soft_upper_limit", upper)))
        limits[joint.get("name")].update({"has_position_limits": True, "min_position": lower + margin, "max_position": upper - margin})
    return moveit_config


# joint_margin (rad) from a params YAML's /** or node section; 0 if not given.
def GetJointMargin(params_file):
    if not params_file:
        return 0.0
    with open(params_file) as f:
        sections = yaml.safe_load(f) or {}
    for section in sections.values():
        margin = ((section or {}).get("ros__parameters") or {}).get("joint_margin")
        if margin is not None:
            return float(margin)
    return 0.0


# Node parameters that follow the robot: planning group and home pose.
def GetRobotParameters(robot):
    return {"group_name": robot["group"], "initial_joints": list(robot["home"].values())}
