from typing import List

MOVE_GROUP_ARM: str = "interbotix_arm"
MOVE_GROUP_GRIPPER: str = "interbotix_gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.037, -0.037]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0195, -0.0195]


def joint_names() -> List[str]:
    return [
        "waist",
        "shoulder",
        "elbow",
        "wrist_angle",
        "wrist_rotate",
        "left_finger",
        "right_finger",
    ]


def base_link_name(prefix: str = "wx200") -> str:
    return prefix + "/base_link"


def end_effector_name(prefix: str = "wx200") -> str:
    return prefix + "/gripper_link"


def gripper_joint_names() -> List[str]:
    return [
        "right_finger",
        "left_finger"

    ]
