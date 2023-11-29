import sys

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from robot_utils import move_arms, torque_on

def puppet_master(side):
    master_bot = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper",
                                              robot_name=f'master_{side}', init_node=True)

    master_sleep_position = (0, -1.1, 1.24, 0, -0.24, 0)
    torque_on(master_bot)
    move_arms([master_bot], [master_sleep_position], move_time=2)

if __name__ == '__main__':
    side = sys.argv[1]
    puppet_master(side)

