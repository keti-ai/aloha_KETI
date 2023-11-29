#!/usr/bin/env python3
import time
import sys
import IPython
import rospy

e = IPython.embed

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import MASTER2PUPPET_JOINT_FN, DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from sensor_msgs.msg import JointState

class OneSideRobotRemoteControl:
    def __init__(self, robot_side, rate=30):
        self.robot_side = robot_side
        self.node_name = f'OneSideRobotRemoteControl_{robot_side}'
        self.puppet_bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper",
                                                  robot_name=f'puppet_{robot_side}', init_node=True)
        self.rate = rospy.Rate(rate)  # 10 Hz
        self.gripper_command = JointSingleCommand(name="gripper")

    def prep_robots(self):
        # reboot gripper motors, and set operating modes for all motors
        self.puppet_bot.dxl.robot_reboot_motors("single", "gripper", True)
        self.puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
        self.puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
        torque_on(self.puppet_bot)

        # move arms to starting position
        start_arm_qpos = START_ARM_POSE[:6]
        move_arms([self.puppet_bot], [start_arm_qpos], move_time=1)
        # move grippers to starting position
        move_grippers([self.puppet_bot], [PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)

    def master_joint_states_updated(self, joint_states):
        rospy.loginfo(f"{self.node_name} joint_state updated: {joint_states}")
        master_state_joints = joint_states.position[:6]
        self.puppet_bot.arm.set_joint_positions(master_state_joints, blocking=False)

        # sync gripper positions
        master_gripper_joint = joint_states.position[6]
        puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(master_gripper_joint)
        self.gripper_command.cmd = puppet_gripper_joint_target
        self.puppet_bot.gripper.core.pub_single.publish(self.gripper_command)

    def run(self):
        rospy.loginfo('Controller node run')
        self.prep_robots()
        rospy.Subscriber(f'/master_{self.robot_side}/joint_states', JointState, self.master_joint_states_updated)
        # while not rospy.is_shutdown():
        #     self.rate.sleep()
        rospy.spin()


if __name__ == '__main__':
    side = sys.argv[1]
    # side = 'right'
    ros_node = OneSideRobotRemoteControl(robot_side=side)
    ros_node.run()
