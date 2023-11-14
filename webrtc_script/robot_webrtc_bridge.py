#!/usr/bin/env python
import json
import sys
from asyncio import AbstractEventLoop
from typing import Callable, Optional
import time
import asyncio
import rospy
from ketirtc_robot import UserClientConnectionManager, ConnectionChannel, UserClient
from ketirtc_robot.webrtc.webrtc_connection_manager import WebRtcConnectionManager
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.msg import JointGroupCommand


class WebRtcBridgeRobot:
    def __init__(self, robot_id, password, server_host="localhost", server_port="8180", camera=None,
                 loop: AbstractEventLoop = None, message_duration_time=300):
        self.loop = loop
        rospy.init_node(f"RobotWebRtcBridge_{robot_id}", anonymous=True)
        rospy.loginfo("webrtc_node is running.")
        self.sleep_time = 0.5
        self.rate = rospy.Rate(100)  # 10 Hz
        self.message_duration_time = message_duration_time
        self.robot_id = robot_id
        self.password = password
        self.server_host = server_host
        self.server_port = server_port
        self.camera = camera

        self.topic_publishers = {}

        self.signalling_server = WebRtcConnectionManager(robot_id=robot_id, signalling_password=password,
                                                         camera=camera,
                                                         signalling_server_host=server_host,
                                                         signalling_server_port=server_port)

        self.user_clients_manager = UserClientConnectionManager()
        self.user_clients_manager.add_connection_manager_channel(self.signalling_server)
        self.user_clients_manager.add_listener('user_message', self._remote_message_received)

    def _handle_topic_message(self, message):
        topic_name, value = message["topic"], message["value"]
        if topic_name not in self.topic_publishers:
            print(f'topic {topic_name} not registered')
            return

        pub, value_parser = self.topic_publishers[topic_name]
        if value_parser is None:
            topic_value = value
        else:
            topic_value = value_parser(value)

        # print(f"pub: {pub}")
        # print(f'publish topic {topic_name}:\n{topic_value}')
        print(f'publish {topic_name}')
        try:
            pub.publish(topic_value)
        except Exception as ex:
            print(f"publish error: {topic_value}")
            print(f"error: {ex}")

        # print(f'end publish {topic_name}')
        print(f'end published topic {topic_name}: {topic_value}')

    def _remote_message_received(self, message, channel: ConnectionChannel, usr_client: UserClient):
        msg_data = json.loads(message)
        print(f'message in json: {msg_data}')
        if "topic" in message:
            self._handle_topic_message(message)
        else:
            print(f'not topic message: {message}')

    def _send_topic_to_remote(self, topic_name, value, json_converter: Optional[Callable] = None):
        if json_converter is None:
            json_value = value
        else:
            json_value = json_converter(value)

        message = {"topic": topic_name,
                   "value": json_value}
        fut = asyncio.ensure_future(self.user_clients_manager.send_to_all_users(message), loop=self.loop)

        def done_sending(task):
            # print(f'sending task done: {task}')
            pass
        fut.add_done_callback(done_sending)
        # print(f'task: {fut}')

    def subscribe_local_topic(self, topic_name, value_type, to_json_converter):
        last_sending_time = None
        def topic_handler(value):
            nonlocal last_sending_time
            # print(f'topic: {topic_name}')
            if last_sending_time is None:
                last_sending_time = time.time()
            
            wait_time = (time.time() - last_sending_time)*1000
            
            if wait_time >= self.message_duration_time:    
                print(f'{topic_name} send. wait_time : {wait_time}')
                self._send_topic_to_remote(topic_name=topic_name, value=value, json_converter=to_json_converter)
                last_sending_time = time.time()
            # else:
            #     print(f'wait_time: wait: {wait_time}')

        rospy.Subscriber(topic_name, value_type, topic_handler)

    def add_topic_publisher(self, topic_name, value_type, value_parser, new_topic_name=None):
        if topic_name in self.topic_publishers:
            print(f'topic already exist: {topic_name}')
            return

        if new_topic_name is None:
            publish_topic_name = topic_name
        else:
            publish_topic_name = new_topic_name

        print(f'add topic {topic_name}, new_topic: {new_topic_name}, type: {value_type}, value_parser: {value_parser}')
        pub = rospy.Publisher(publish_topic_name, value_type, queue_size=20)
        self.topic_publishers[topic_name] = (pub, value_parser)

    async def run(self):
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
        if self.loop is None:
            self.loop = asyncio.new_event_loop()

        self.user_clients_manager.start()
        while not rospy.is_shutdown():
            self.rate.sleep()
            await asyncio.sleep(self.sleep_time, loop=self.loop)

    async def stop(self):
        self.user_clients_manager.stop()


if __name__ == '__main__':
    robot_id = sys.argv[1]
    server_host = sys.argv[2]
    server_port = sys.argv[3]

    event_loop = asyncio.get_event_loop()
    bridgeNode = None
    try:
        # robot_id = "syscon_aloha_puppet_P02"
        password = ""

        # server_host = "175.126.123.199"
        # server_host = "localhost"
        # server_port = "8180"

        from ketirtc_robot.camera.cv2_camera import CV2StreamCamera
        camera = CV2StreamCamera(camera_index=0)

        # from webrtc_script.ros_camera import ROSCV2StreamCamera
        # camera_topics = "/usb_cam_high/image_raw/compressed"
        # camera = ROSCV2StreamCamera(camera_topics)

        bridgeNode = WebRtcBridgeRobot(robot_id, password, server_host=server_host, server_port=server_port,
                                       camera=camera, loop=event_loop)

        from utils import joint_state_to_json, joint_group_command_to_json, joint_single_command_to_json, \
            json_to_joint_state, json_to_joint_group_command, json_to_joint_single_command

        bridgeNode.subscribe_local_topic(topic_name="/master_left/joint_states", value_type=JointState,
                                         to_json_converter=joint_state_to_json)

        bridgeNode.add_topic_publisher(topic_name="/master_left/joint_states",
                                       value_type=JointState,
                                       value_parser=json_to_joint_state)

        bridgeNode.add_topic_publisher(topic_name="/master_right/joint_states",
                                       value_type=JointState,
                                       value_parser=json_to_joint_state)

        event_loop.run_until_complete(bridgeNode.run())

    except rospy.ROSInterruptException:
        print(f'rospy.ROSInterruptException')
        pass
    
    if bridgeNode:
            event_loop.run_until_complete(bridgeNode.stop())
    event_loop.close()
