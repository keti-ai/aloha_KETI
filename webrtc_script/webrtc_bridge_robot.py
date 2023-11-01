#!/usr/bin/env python
import json
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
        rospy.init_node(f"WebRtcBridge_{robot_id}", anonymous=True)
        rospy.loginfo("webrtc_node is running.")
        self.sleep_time = 0.5
        self.rate = rospy.Rate(100)  # 10 Hz
        self.message_duration_time = message_duration_time
        self.robot_id = robot_id
        self.password = password
        self.server_host = server_host
        self.server_port = server_port
        self.camera = camera

        self.signalling_server = WebRtcConnectionManager(robot_id=robot_id, signalling_password=password,
                                                         camera=camera,
                                                         signalling_server_host=server_host,
                                                         signalling_server_port=server_port)

        self.user_clients_manager = UserClientConnectionManager()
        self.user_clients_manager.add_connection_manager_channel(self.signalling_server)
        self.user_clients_manager.add_listener('user_message', self._remote_message_received)

    def _remote_message_received(self, message, channel: ConnectionChannel, usr_client: UserClient):
        msg_data = json.loads(message)
        print(f'message in json: {msg_data}')

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
            
            wait_time =  (time.time() - last_sending_time)*1000
            
            if wait_time >= self.message_duration_time:    
                print(f'{topic_name} send. wait_time : {wait_time}')
                self._send_topic_to_remote(topic_name=topic_name, value=value, json_converter=to_json_converter)
                last_sending_time = time.time()
            # else:
            #     print(f'wait_time: wait: {wait_time}')

        rospy.Subscriber(topic_name, value_type, topic_handler)

        # rospy.Subscriber(topic_name, value_type,
        #                  lambda value, topic=topic_name, json_converter=to_json_converter: self._send_topic_to_remote(
        #                      topic, value, to_json_converter))

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
    event_loop = asyncio.get_event_loop()
    bridgeNode = None
    try:
        robot_id = "syscon_example_P02"
        password = ""

        server_host = "175.126.123.199"
        # server_host = "localhost"
        server_port = "8180"

        from ketirtc_robot.camera.cv2_camera import CV2StreamCamera
        camera = CV2StreamCamera(camera_index=0)

        bridgeNode = WebRtcBridgeRobot(robot_id, password, server_host=server_host, server_port=server_port,
                                       camera=camera, loop=event_loop)

        from utils import joint_state_to_json, joint_group_command_to_json, joint_single_command_to_json
        # bridgeNode.subscribe_local_topic(topic_name="/master_left/joint_states", value_type=JointState,
        #                                  to_json_converter=joint_state_to_json)
        

        bridgeNode.subscribe_local_topic(topic_name="/puppet_left/joint_states", 
                                         value_type=JointState,
                                         to_json_converter=joint_state_to_json)
        
        bridgeNode.subscribe_local_topic(topic_name="/puppet_left/commands/joint_group", 
                                         value_type=JointGroupCommand,
                                         to_json_converter=joint_group_command_to_json)
        
        bridgeNode.subscribe_local_topic(topic_name="/puppet_left/commands/joint_signle", value_type=JointSingleCommand,
                                         to_json_converter=joint_single_command_to_json)
        
        event_loop.run_until_complete(bridgeNode.run())

    except rospy.ROSInterruptException:
        print(f'rospy.ROSInterruptException')
        pass
    
    if bridgeNode:
            event_loop.run_until_complete(bridgeNode.stop())
    event_loop.close()