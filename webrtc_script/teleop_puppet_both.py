import json
import time
import sys
import IPython
e = IPython.embed
import rospy
from std_msgs.msg import String

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import MASTER2PUPPET_JOINT_FN, DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions

from tele_op_robot import TeleOpRobot

puppet_bot_left = None
puppet_bot_right = None
gripper_command = None

master_state_joints_left = None
master_gripper_joint_left = None

master_state_joints_right = None
master_gripper_joint_right = None


def prep_robots(puppet_bot):
    # reboot gripper motors, and set operating modes for all motors
    puppet_bot.dxl.robot_reboot_motors("single", "gripper", True)
    puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")

    torque_on(puppet_bot)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms([puppet_bot], [start_arm_qpos] * 2, move_time=1)
    # move grippers to starting position
    move_grippers([puppet_bot], [PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)


def teleop():
    global puppet_bot_left
    global puppet_bot_right
    global gripper_command

    global master_state_joints_left
    global master_gripper_joint_left

    global master_state_joints_right
    global master_gripper_joint_right

    """ A standalone function for experimenting with teleoperation. No data recording. """
    puppet_bot_left = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_left', init_node=True)
    print('passed')
    puppet_bot_right = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_right', init_node=False)

    prep_robots(puppet_bot_left)
    prep_robots(puppet_bot_right)

    ### Teleoperation loop
    gripper_command = JointSingleCommand(name="gripper")

    while True:
        if master_state_joints_left:
            puppet_bot_left.arm.set_joint_positions(master_state_joints_left, blocking=False)
        if master_state_joints_right:
            puppet_bot_right.arm.set_joint_positions(master_state_joints_right, blocking=False)
        if master_gripper_joint_left:
            puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(master_gripper_joint_left)
            gripper_command.cmd = puppet_gripper_joint_target
            puppet_bot_left.gripper.core.pub_single.publish(gripper_command)
        if master_gripper_joint_right:
            puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(master_gripper_joint_right)
            gripper_command.cmd = puppet_gripper_joint_target
            puppet_bot_right.gripper.core.pub_single.publish(gripper_command)
        # sleep DT
        # time.sleep(DT)


def update_joint_state(data):

    global puppet_bot_left
    global puppet_bot_right
    global gripper_command

    global master_state_joints_left
    global master_gripper_joint_left

    global master_state_joints_right
    global master_gripper_joint_right

    data = json.loads(data.data)
    print(data)

    if 'left' in data['msg_body']:
        master_state_joints_left = data['msg_body']['left']['master_state_joints']
        master_gripper_joint_left = data['msg_body']['left']['master_gripper_joint']

    if 'right' in data['msg_body']:
        master_state_joints_right = data['msg_body']['right']['master_state_joints']
        master_gripper_joint_right = data['msg_body']['right']['master_gripper_joint']



if __name__ == '__main__':
    rospy.Subscriber('/tele_operator/joint_state', String, update_joint_state)
    teleop()
    rospy.spin()