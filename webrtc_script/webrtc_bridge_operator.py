#!/usr/bin/env python
import json
from asyncio import AbstractEventLoop
from typing import Callable, Dict, Optional

import asyncio
import rospy
from ketirtc_robot import UserClientConnectionManager, ConnectionChannel, UserClient
from ketirtc_robot.webrtc.webrtc_connection_manager import WebRtcConnectionManager
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.msg import JointGroupCommand
from aiortc.contrib.media import MediaRecorder, MediaBlackhole
from ketirtc_operator.RemoteRobotController import RemoteRobotController

class WebRtcBridgeOperator:
    def __init__(self, robot_id, password, server_host="localhost", server_port="8180",
                 loop: AbstractEventLoop = None):
        self.loop = loop
        rospy.init_node(f"WebRtcBridgeOperator_{robot_id}", anonymous=True)
        rospy.loginfo("webrtc_node is running.")
        self.sleep_time = 0.1
        self.rate = rospy.Rate(10)  # 10 Hz
        self.target_robot_id = robot_id
        self.password = password
        self.server_host = server_host
        self.server_port = server_port

        # camera_recorder = MediaRecorder(f"{target_robot_id}.mp4")
        self.camera_recorder = MediaBlackhole()
        self.remote_robot = RemoteRobotController(self.target_robot_id, server_host, server_port, user_id="python_user1",
                                         recorder=self.camera_recorder)

        self.remote_robot.set_message_received_handler(self.handle_robot_message)
        # self.topic_publishers: Dict[str, (rospy.Publisher, Callable)] = {}
        self.topic_publishers = {}

    async def handle_robot_message(self, msg, channel):
        # print(f"robot message received: {msg}")
        message = json.loads(msg)
        # print(f"robot message received: {message}")
        if "topic" in message:
            self._handle_topic_message(message)
        else:
            print(f'not topic message: {message}')

    def _handle_topic_message(self, message):
        topic_name, value  = message["topic"], message["value"]
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

    def _remote_message_received(self, message, channel: ConnectionChannel, usr_client: UserClient):
        # print(f'message from user {usr_client.client_id} via channel "{channel.channel_name}": {message}')
        msg_data = json.loads(message)
        print(f'message in json: {msg_data}')
        # TODO: parse get topic message and publish

    def _send_topic_to_remote(self, topic_name, value, json_converter: Optional[Callable] = None):
        if json_converter is None:
            json_value = value
        else:
            json_value = json_converter(value)

        message = {"topic": topic_name,
                   "value": json_value}
        # TODO: send to remote
        # self.loop.run_until_complete(self.user_clients_manager.send_to_all_users(message=message))
        # self.user_clients_manager.send_to_all_users(message)
        fut = asyncio.ensure_future(self.remote_robot.send_json(message), loop=self.loop)

        def done_sending(task):
            print(f'sending task done: {task}')
        fut.add_done_callback(done_sending)
        # print(f'task: {fut}')

    def subscribe_local_topic(self, topic_name, value_type, to_json_converter):
        rospy.Subscriber(topic_name, value_type,
                         lambda value, topic=topic_name, json_converter=to_json_converter: self._send_topic_to_remote(
                             topic, value, to_json_converter))

    async def run(self):
        if self.loop is None:
            self.loop = asyncio.get_event_loop()
        if self.loop is None:
            self.loop = asyncio.new_event_loop()

        task = asyncio.create_task(self.remote_robot.connect())
        while not rospy.is_shutdown():
            # await asyncio.sleep(self.sleep_time, loop=self.loop)
            await asyncio.sleep(1, loop=self.loop)
            self.rate.sleep()
            
        task.cancel()
        print(f'end run')

    async def stop(self):
        await self.remote_robot.close()


if __name__ == '__main__':
    import logging
    def config_logging():
        # format = '%(asctime)s,%(msecs)03d:[%(levelname)]:[%(filename)s:%(lineno)d] %(message)s'
        # logging.basicConfig(level=logging.DEBUG, datefmt='%Y-%m-%d:%H:%M:%S', format=format)
        logging.basicConfig(level=logging.DEBUG)
        logging.getLogger("aiortc").setLevel(level=logging.INFO)
        logging.getLogger("aioice").setLevel(level=logging.INFO)
        

    config_logging()

    event_loop = asyncio.get_event_loop()
    bridgeNode = None
    try:
        robot_id = "syscon_example_P02"
        password = ""

        server_host = "175.126.123.199"
        # server_host = "localhost"
        server_port = "8180"


        bridgeNode = WebRtcBridgeOperator(robot_id, password, server_host=server_host, server_port=server_port, loop=event_loop)

        from utils import joint_state_to_json, json_to_joint_state, json_to_joint_group_command, \
            json_to_joint_single_command

        bridgeNode.subscribe_local_topic(topic_name="/master_left/joint_states", value_type=JointState,
                                         to_json_converter=joint_state_to_json)

        # topic_name="/master_left/joint_states"
        # publish_topic_name = f'/forward_{topic_name}'
        # bridgeNode.add_topic_publisher(topic_name=topic_name, new_topic_name=publish_topic_name, value_type=JointState, value_parser=json_to_joint_state)
        
        # topic_name="/puppet_left/joint_states"
        # publish_topic_name = f'/forward_{topic_name}'
        # bridgeNode.add_topic_publisher(topic_name=topic_name,
        #                                new_topic_name=publish_topic_name,
        #                                value_type=JointState,
        #                                value_parser=json_to_joint_state)
        #
        #
        # topic_name="/puppet_left/commands/joint_group"
        # publish_topic_name = f'/forward_{topic_name}'
        # bridgeNode.add_topic_publisher(topic_name=topic_name,
        #                                new_topic_name=publish_topic_name,
        #                                value_type=JointGroupCommand,
        #                                value_parser=json_to_joint_group_command)
        #
        # topic_name="/puppet_left/commands/joint_signle"
        # publish_topic_name = f'/forward_{topic_name}'
        # bridgeNode.add_topic_publisher(topic_name=topic_name,
        #                                new_topic_name=publish_topic_name,
        #                                value_type=JointSingleCommand,
        #                                value_parser=json_to_joint_single_command)

        event_loop.run_until_complete(bridgeNode.run())

    except rospy.ROSInterruptException:
        print(f'rospy.ROSInterruptException')
        pass

    if bridgeNode:
        event_loop.run_until_complete(bridgeNode.stop())
            
    event_loop.close()