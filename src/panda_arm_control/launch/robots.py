"""Per-robot settings shared by the launch files: robot:=panda (default) or robot:=iiwa14."""
from moveit_configs_utils import MoveItConfigsBuilder

# home: the tour's start pose (overrides object_pose.yaml's initial_joints). The iiwa14 home puts tool0 at the Panda
# home's tool0 pose, so both robots start the tour from the same camera pose.
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
    "iiwa14": {
        "name": "iiwa14",
        "package": "iiwa_arm_moveit",
        "urdf": "urdf/iiwa14.urdf.xacro",
        "srdf": "config/iiwa14.srdf",
        "controllers": "config/moveit_controllers.yaml",
        "group": "arm",
        "base_link": "lbr_link_0",
        "home": {"lbr_A1": 0.0, "lbr_A2": -0.3099, "lbr_A3": 0.0, "lbr_A4": -1.9252, "lbr_A5": 0.0, "lbr_A6": 1.4367, "lbr_A7": 0.0},
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


# Node parameters that follow the robot: planning group and home pose.
def GetRobotParameters(robot):
    return {"group_name": robot["group"], "initial_joints": list(robot["home"].values())}
