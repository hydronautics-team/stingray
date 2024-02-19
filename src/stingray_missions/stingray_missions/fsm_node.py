import asyncio

import rclpy
from rclpy.node import Node

from stingray_missions.fsm import FSM


async def ros_loop(node: Node):
    """ROS loop for spinning the node"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.1)


async def fsm_loop(node: Node, fsm: FSM):
    """FSM loop"""
    fsm.load_scenarios_from_packages(package_names=node.get_parameter('package_names').get_parameter_value().string_array_value)
    # trigger transition service
    while rclpy.ok():
        await fsm.process_pending_transition()
        await asyncio.sleep(0.1)


# async def execution_loop(fsm: FSM):
#     """Loop for executing grasp"""
#     while rclpy.ok():
#         if not fsm.pending_transition:
#             await fsm.process_grasps()
#         await asyncio.sleep(0.1)


def main():
    rclpy.init()

    node = rclpy.create_node('stingray_missions')
    node.declare_parameter(name="package_names", value=["stingray_missions"])
    fsm = FSM(node=node)
    event_loop = asyncio.get_event_loop()
    future = asyncio.wait(
        [ros_loop(node), fsm_loop(node, fsm)], return_when=asyncio.FIRST_EXCEPTION
    )
    done, _pending = event_loop.run_until_complete(future)
    for task in done:
        task.result()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
