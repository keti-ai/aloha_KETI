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
from webrtc_bridge_robot import WebRtcBridgeRobot


if __name__ == '__main__':
    event_loop = asyncio.get_event_loop()
    bridgeNode = None
    try:
        robot_id = "syscon_puppet_left_P02"
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
        
        bridgeNode.subscribe_local_topic(topic_name="/puppet_left/commands/joint_signle", 
                                         value_type=JointSingleCommand,
                                         to_json_converter=joint_single_command_to_json)
        
        event_loop.run_until_complete(bridgeNode.run())

    except rospy.ROSInterruptException:
        print(f'rospy.ROSInterruptException')
        pass
    
    if bridgeNode:
            event_loop.run_until_complete(bridgeNode.stop())
    event_loop.close()