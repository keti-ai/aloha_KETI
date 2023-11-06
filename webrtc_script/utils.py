
import json
from sensor_msgs.msg import JointState

from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.msg import JointGroupCommand

def joint_state_to_json(joint_state):
    """
    Convert a sensor_msgs.msg.JointState object to JSON.

    :param joint_state: sensor_msgs.msg.JointState object
    :return: JSON representation of the JointState
    """
    joint_state_dict = {
        'header': {
            'seq': joint_state.header.seq,
            'stamp': {'secs': joint_state.header.stamp.secs, 'nsecs': joint_state.header.stamp.nsecs},
            'frame_id': joint_state.header.frame_id
        },
        'name': joint_state.name,
        'position': joint_state.position,
        'velocity': joint_state.velocity,
        'effort': joint_state.effort
    }

    # return json.dumps(joint_state_dict)
    return joint_state_dict

def json_to_joint_state(joint_state_dict):
    """
    Convert JSON representation back to a sensor_msgs.msg.JointState object.

    :param json_str: JSON representation of JointState
    :return: sensor_msgs.msg.JointState object
    """
    # joint_state_dict = json.loads(json_str)
    joint_state = JointState()
    
    joint_state.header.seq = joint_state_dict['header']['seq']
    joint_state.header.stamp.secs = joint_state_dict['header']['stamp']['secs']
    joint_state.header.stamp.nsecs = joint_state_dict['header']['stamp']['nsecs']
    joint_state.header.frame_id = joint_state_dict['header']['frame_id']

    joint_state.name = joint_state_dict['name']
    joint_state.position = joint_state_dict['position']
    joint_state.velocity = joint_state_dict['velocity']
    joint_state.effort = joint_state_dict['effort']

    return joint_state


def joint_group_command_to_json(group_command):
    group_command_dict = {
        'cmd': group_command.cmd,
        'name': group_command.name
    }
    return group_command_dict
    # return json.dumps(group_command_dict)


def json_to_joint_group_command(joint_state_dict):
    joint_state = JointGroupCommand()
    
    joint_state.cmd = joint_state_dict['cmd']
    joint_state.name = joint_state_dict['name']

    return joint_state


def joint_single_command_to_json(group_command):
    group_command_dict = {
        'cmd': group_command.cmd,
        'name': group_command.name
    }
    return group_command_dict
    # return json.dumps(group_command_dict)


def json_to_joint_single_command(joint_state_dict):
    print(f'json to single_command: {joint_state_dict}')
    joint_state = JointSingleCommand()
    
    joint_state.cmd = joint_state_dict['cmd']
    joint_state.name = joint_state_dict['name']

    return joint_state