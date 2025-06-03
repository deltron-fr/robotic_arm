import numpy as np

from ikpy.chain import Chain


def inverse_kinematics(target):
    np.set_printoptions(suppress=True, precision=10)
    arm_chain = Chain.from_urdf_file("robot.urdf")

    joint_angles = np.degrees(arm_chain.inverse_kinematics(target))

    return list(map(lambda x: round(x, 2), joint_angles))

    # print(arm_chain.forward_kinematics(joint_angles))
