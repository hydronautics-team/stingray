import asyncio

import rclpy
from rclpy.node import Node

from stingray_missions.fsm import FSM


async def ros_loop(node: Node):
    """ROS loop for spinning the node"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1e-4)


async def fsm_loop(fsm: FSM):
    """FSM loop"""
    fsm.draw()
    # trigger transition service
    while rclpy.ok():
        await fsm.process_pending_transition()
        await asyncio.sleep(1e-4)


async def state_action_loop(fsm: FSM):
    """Loop for executing grasp"""
    while rclpy.ok():
        await fsm.process_pending_action()
        await asyncio.sleep(1e-4)


def main():
    rclpy.init()

    node = rclpy.create_node('stingray_missions')
    node.declare_parameter(name="mission_package_names", value=['stingray_missions'])
    mission_package_names = node.get_parameter('mission_package_names').get_parameter_value().string_array_value
    fsm = FSM(node=node, scenarios_packages=mission_package_names)
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
