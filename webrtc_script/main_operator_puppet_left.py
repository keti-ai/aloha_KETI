#!/usr/bin/env python

import json
from asyncio import AbstractEventLoop
from typing import Callable, Optional
import asyncio
import rospy
from ketirtc_robot.webrtc.webrtc_connection_manager import WebRtcConnectionManager
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.msg import JointGroupCommand
from webrtc_bridge_operator import WebRtcBridgeOperator


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
        robot_id = "syscon_puppet_left_P02"
        password = ""

        server_host = "175.126.123.199"
        # server_host = "localhost"
        server_port = "8180"


        bridgeNode = WebRtcBridgeOperator(robot_id, password, server_host=server_host, server_port=server_port, loop=event_loop)

        from utils import json_to_joint_state, json_to_joint_group_command, json_to_joint_single_command

        topic_name="/puppet_left/joint_states"
        publish_topic_name = f'/forward_{topic_name}'
        bridgeNode.add_topic_publisher(topic_name=topic_name, 
                                       new_topic_name=publish_topic_name, 
                                       value_type=JointState, 
                                       value_parser=json_to_joint_state)
        

        topic_name="/puppet_left/commands/joint_group"
        publish_topic_name = f'/forward_{topic_name}'
        bridgeNode.add_topic_publisher(topic_name=topic_name, 
                                       new_topic_name=publish_topic_name, 
                                       value_type=JointGroupCommand, 
                                       value_parser=json_to_joint_group_command)
        
        topic_name="/puppet_left/commands/joint_signle"
        publish_topic_name = f'/forward_{topic_name}'
        bridgeNode.add_topic_publisher(topic_name=topic_name, 
                                       new_topic_name=publish_topic_name, 
                                       value_type=JointSingleCommand, 
                                       value_parser=json_to_joint_single_command)
        
        event_loop.run_until_complete(bridgeNode.run())

    except rospy.ROSInterruptException:
        print(f'rospy.ROSInterruptException')
        pass

    if bridgeNode:
        event_loop.run_until_complete(bridgeNode.stop())
            
    event_loop.close()
    