
import rospy
from sensor_msgs.msg import JointState

from aloha_scripts.constants import MASTER2PUPPET_JOINT_FN


class RemoteMasterBot:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self._joint_states = None
        rospy.Subscriber(f'/{self.robot_name}/joint_states', JointState, self._joint_states_updated)

    def _joint_states_updated(self, joint_states):
        # print(f'Robot {self.robot_name} joint_states updated: {joint_states}')
        self._joint_states = joint_states

    @property
    def joint_states(self):
        return self._joint_states

    def get_arm_joint_positions(self):
        if self.joint_states is None:
            return None
        return self.joint_states.position[:6]

    def get_arm_gripper_positions(self):
        if self.joint_states is None:
            return None
        return self.joint_states.position[6]

    def move_arms(self):
        pass

    def move_grippers(self):
        pass

    def torque_off(bot):
        pass

    def torque_on(bot):
        pass
