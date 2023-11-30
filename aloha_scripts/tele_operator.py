import asyncio
import logging
import threading
import time

from aiortc.contrib.media import MediaRecorder, MediaBlackhole
from ketirtc_operator.RemoteRobotController import RemoteRobotController

import rospy
from sensor_msgs.msg import JointState  # JointState 메시지 타입 임포트

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

logging.getLogger("aiortc").setLevel(logging.INFO)
logging.getLogger("aioice").setLevel(logging.INFO)

class TeleOperator:
    def __init__(self, target_robot_id, server_host, server_port, user_id="python_user1"):
        self.target_robot_id = target_robot_id
        self.server_host = server_host
        self.server_port = server_port
        self.user_id = user_id
        self.message = None
        self.camera_recorder = MediaBlackhole()
        self.remote_robot = RemoteRobotController(self.target_robot_id, self.server_host, self.server_port, user_id=self.user_id, recorder=self.camera_recorder)
        self.remote_robot.set_message_received_handler(self.handle_robot_message)

        self.msg_init()


        rospy.init_node('tele_operator_node')
        rospy.Subscriber('/master_left/joint_states', JointState, self.left_msg)
        rospy.Subscriber('/master_right/joint_states', JointState, self.right_msg)

    async def handle_robot_message(self, message, channel):
        print(f"robot message received: {message}")
        logger.info(f"robot message received: {message}")


    def left_msg(self, data):
        state_joints = list(data.position)
        self.message['msg_body']['left'] = {
            "master_state_joints": state_joints[:6],
            "master_gripper_joint": state_joints[6]
        }

    def right_msg(self, data):
        state_joints = list(data.position)
        self.message['msg_body']['right'] = {
            "master_state_joints": state_joints[:6],
            "master_gripper_joint": state_joints[6]
        }


    def msg_init(self):
        self.message = {
            'msg_type': 'joint_state',
            'msg_body': {}
        }

    def run(self):
        loop = asyncio.get_event_loop()
        try:
            thread = threading.Thread(name=f'Thread', target=loop.run_until_complete, args=([self.remote_robot.connect()]))
            thread.start()

            while True:
                try:
                    if 'left' in self.message['msg_body'] and 'right' in self.message['msg_body']:
                        self.remote_robot.send_json(self.message)
                        # print(self.message)
                        time.sleep(0.005)
                        self.msg_init()

                except Exception as e:
                    print(e)



        except KeyboardInterrupt:
            pass
        finally:
            asyncio.ensure_future(self.remote_robot.close(), loop=loop)
        print('exited!')

if __name__ == '__main__':
    controller = TeleOperator('syscon_aloha_kh', '175.126.123.199', 8180)
    controller.run()