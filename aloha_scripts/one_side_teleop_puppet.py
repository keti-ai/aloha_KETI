import time
import sys
import IPython
import rospy
from sensor_msgs.msg import JointState

e = IPython.embed

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import MASTER2PUPPET_JOINT_FN, DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions

def prep_robots(puppet_bot):
    # reboot gripper motors, and set operating modes for all motors
    puppet_bot.dxl.robot_reboot_motors("single", "gripper", True)
    puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    # puppet_bot.dxl.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit
    torque_on(puppet_bot)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms([puppet_bot], [start_arm_qpos], move_time=1)
    # move grippers to starting position
    move_grippers([puppet_bot], [PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)

def teleop(robot_side):
    """ A standalone function for experimenting with teleoperation. No data recording. """
    puppet_bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_{robot_side}', init_node=True)

    prep_robots(puppet_bot)

    ### Teleoperation loop
    gripper_command = JointSingleCommand(name="gripper")

    def master_joint_states_update_callback(joint_states):
        rospy.loginfo(f"joint_state updated: {joint_states}")
        master_state_joints = joint_states.position[:6]
        puppet_bot.arm.set_joint_positions(master_state_joints, blocking=False)

        # sync gripper positions
        master_gripper_joint = joint_states.position[6]
        puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(master_gripper_joint)
        gripper_command.cmd = puppet_gripper_joint_target
        puppet_bot.gripper.core.pub_single.publish(gripper_command)

    rospy.Subscriber(f'/master_{robot_side}/joint_states', JointState, master_joint_states_update_callback)
    # while not rospy.is_shutdown():
    #     self.rate.sleep()
    rospy.spin()


if __name__=='__main__':
    side = sys.argv[1]
    # side = 'right'
    teleop(side)
