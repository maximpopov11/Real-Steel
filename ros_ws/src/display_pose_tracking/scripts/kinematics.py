from typing import Tuple


# arm_joint_angles is representative of our g1's arm
# TODO: we need to make this of the correct size
arm_joint_angles = Tuple[
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
]


def kinematics(points) -> arm_joint_angles:
    pass
