import asyncio
import json
from std_msgs.msg import String

from ketirtc_robot import UserClient, ConnectionChannel
from ketirtc_robot.camera.cv2_camera import CV2StreamCamera
from ketirtc_robot.camera import get_local_stream_camera
from ketirtc_robot.connection_manager import UserClientConnectionManager
from ketirtc_robot.webrtc.webrtc_connection_manager import WebRtcConnectionManager
from aloha_scripts.aloha_multi_cam import MultiCam

from pubsub import pub

import rospy
from sensor_msgs.msg import JointState  # JointState 메시지 타입 임포트



class TeleOpRobot:
    def __init__(self, robot_id='proj_aloha', cam='CV2CAMERA', cam_index=0):
        self.robot_id = robot_id
        self.password = ""

        self.server_host = "175.126.123.199"
        self.server_port = "8180"

        self.message = None

        if cam != 'MULTI_CAM':
            self.camera = get_local_stream_camera(cam, cam_index)
        else:
            self.camera = MultiCam(cam_index)

        self.loop = asyncio.get_event_loop()
        self.joint_state = None

        rospy.init_node('tele_robot_node')
        self.state_publisher = rospy.Publisher('/tele_operator/joint_state', String, queue_size=10)

        self.msg_init()

    def start(self):
        self.signalling_server = WebRtcConnectionManager(robot_id=self.robot_id, signalling_password=self.password,
                                                    camera=self.camera,
                                                    signalling_server_host=self.server_host,
                                                    signalling_server_port=self.server_port)

        self.user_clients_manager = UserClientConnectionManager()
        self.user_clients_manager.add_connection_manager_channel(self.signalling_server)

        # setup event handlers
        self.user_clients_manager.add_listener('user_connected', self.user_connected_handler)
        self.user_clients_manager.add_listener('user_disconnected', self.user_disconnected_handler)
        self.user_clients_manager.add_listener('user_message', self.user_message_handler)

        rospy.Subscriber('/puppet_left/joint_states', JointState, self.left_msg)
        rospy.Subscriber('/puppet_right/joint_states', JointState, self.right_msg)

        print('started')
        self.user_clients_manager.start()
        self.loop.run_until_complete(self.send_messages_to_user())

    def user_connected_handler(self, usr_client: UserClient):
        print(f'new user client connected: user_id: {usr_client.client_id}, {usr_client}')
        print(f'# of user clients: {self.user_clients_manager.clients_count}')

    def user_disconnected_handler(self, usr_client: UserClient):
        print(f'user client disconnected: user_id: {usr_client.client_id}, {usr_client}')
        print(f'# of user clients: {self.user_clients_manager.clients_count}')

    def user_message_handler(self, message, channel: ConnectionChannel, usr_client: UserClient):
        print(f'message from user {usr_client.client_id} via channel "{channel.channel_name}": {message}')
        msg_data = json.loads(message)
        if 'msg_type' in msg_data:
            if msg_data['msg_type'] == 'joint_state':
                self.state_publisher.publish(message)

    # def publish_joint_state(self, joint_state_message):
    #     """
    #     Publish a joint state message to the /tele_operator/joint_state topic.
    #
    #     :param joint_state_message: A dictionary containing the joint state information.
    #     """
    #     self.joint_state = joint_state_message
    #     # 메시지를 JSON 문자열로 변환
    #     message_json = json.dumps(joint_state_message)
    #     # ROS 메시지 생성
    #     ros_message = String(data=message_json)
    #     # 토픽에 메시지 퍼블리시
    #     self.state_publisher.publish(ros_message)
    #     print("Published joint state message:", message_json)

    def left_msg(self, data):
        state_joints = list(data.position)
        self.message['msg_body']['left'] = {
            "puppet_state_joints": state_joints[:6],
            "puppet_gripper_joint": state_joints[6]
        }

    def right_msg(self, data):
        state_joints = list(data.position)
        self.message['msg_body']['right'] = {
            "puppet_state_joints": state_joints[:6],
            "puppet_gripper_joint": state_joints[6]
        }

    def msg_init(self):
        self.message = {
            'msg_type': 'joint_state',
            'msg_body': {}
        }

    async def send_messages_to_user(self):
        while True:
            if 'left' in self.message['msg_body'] and 'right' in self.message['msg_body']:
                # await self.user_clients_manager.send_to_all_users(self.message)
                # print(self.message)
                self.msg_init()
            await asyncio.sleep(0.1)


if __name__ == '__main__':
    tele_robot = TeleOpRobot(robot_id='syscon_test', cam='MULTI_CAM', cam_index=[2, 0, 4, 6])
    # tele_robot = TeleOpRobot(robot_id='syscon_test', cam='CV2CAMERA', cam_index=None)
    tele_robot.start()
