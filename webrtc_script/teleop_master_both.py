import threading
import time
import sys
import IPython
e = IPython.embed

from pubsub import pub
from tele_operator import TeleOperator

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import MASTER2PUPPET_JOINT_FN, DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions


def prep_robots(master_bot):
    # reboot gripper motors, and set operating modes for all motors

    master_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    master_bot.dxl.robot_set_operating_modes("single", "gripper", "position")

    torque_on(master_bot)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms([master_bot], [start_arm_qpos] * 1, move_time=1)
    # move grippers to starting position
    move_grippers([master_bot], [MASTER_GRIPPER_JOINT_MID], move_time=0.5)


def press_to_start(master_bot):
    # press gripper to start data collection
    # disable torque for only gripper joint of master robot to allow user movement
    master_bot.dxl.robot_torque_enable("single", "gripper", False)
    print(f'Close the gripper to start')
    close_thresh = -0.3
    pressed = False
    while not pressed:
        gripper_pos = get_arm_gripper_positions(master_bot)
        if gripper_pos < close_thresh:
            pressed = True
        time.sleep(DT/10)
    torque_off(master_bot)
    print(f'Started!')


def teleop():
    """ A standalone function for experimenting with teleoperation. No data recording. """
    master_bot_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name='master_left', init_node=True)
    print(f'left_init:{master_bot_left}')
    master_bot_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name='master_right', init_node=False)
    print(f'right_init:{master_bot_right}')

    prep_robots(master_bot_left)
    prep_robots(master_bot_right)
    press_to_start(master_bot_left)
    press_to_start(master_bot_right)

    ### Teleoperation loop
    # gripper_command = JointSingleCommand(name="gripper")
    while True:

        time.sleep(DT)


if __name__=='__main__':
    teleop()
    thread = threading.Thread(name=f'Thread', target=teleop)
    thread.start()
    controller = TeleOperator('proj_aloha', '175.126.123.199', 8180)
    controller.run()
