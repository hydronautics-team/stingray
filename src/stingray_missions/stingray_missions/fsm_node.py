import asyncio

import rclpy
from rclpy.node import Node

from stingray_missions.fsm import FSM
from stingray_missions.action import load_stingray_actions


async def ros_loop(node: Node):
    """ROS loop for spinning the node"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1e-4)


async def fsm_loop(fsm: FSM):
    """FSM loop"""
    # fsm.draw()
    # trigger transition service
    while rclpy.ok():
        await fsm.process_pending_transition()
        await asyncio.sleep(1e-4)


async def state_action_loop(fsm: FSM):
    """Loop for executing grasp"""
    while rclpy.ok():
        await fsm.process_pending_action()
        await asyncio.sleep(1e-4)


def declare_parameters(node: Node):
    node.declare_parameter("mission_package_names", [
        'sauvc_missions, stingray_missions'])
    node.declare_parameter(
        'uv_state_topic', '/stingray/topics/uv_state')
    node.declare_parameter('twist_action', '/stingray/actions/twist')
    node.declare_parameter(
        'bbox_search_twist_action', '/stingray/actions/bbox_search_twist')
    node.declare_parameter(
        'bbox_centering_twist_action', '/stingray/actions/bbox_centering_twist')
    node.declare_parameter(
        'device_action', '/stingray/actions/device')
    node.declare_parameter(
        'reset_imu_srv', '/stingray/services/reset_imu')
    node.declare_parameter(
        'transition_srv', '/stingray/services/transition')
    node.declare_parameter(
        'set_stabilization_srv', '/stingray/services/set_stabilization')
    node.declare_parameter(
        'enable_object_detection_topic', '/stingray/topics/enable_object_detection')
    node.declare_parameter(
        'set_recording_srv', '/stingray/services/set_recording_srv')


def main():
    rclpy.init()

    node = rclpy.create_node('sauvc_missions')

    declare_parameters(node)

    mission_package_names = node.get_parameter(
        'mission_package_names').get_parameter_value().string_array_value
    fsm = FSM(node=node, scenarios_packages=mission_package_names,
              actions=load_stingray_actions(node))
    event_loop = asyncio.get_event_loop()
    future = asyncio.wait(
        [ros_loop(node), fsm_loop(fsm), state_action_loop(fsm)], return_when=asyncio.FIRST_EXCEPTION
    )
    done, _pending = event_loop.run_until_complete(future)
    for task in done:
        task.result()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
