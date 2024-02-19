import asyncio

import rclpy
from rclpy.node import Node

from stingray_missions.fsm import FSM, load_scenario


async def ros_loop(node: Node):
    """ROS loop for spinning the node"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.1)


async def fsm_loop(fsm: FSM):
    """FSM loop"""
    fsm.initialize(scenario=load_scenario("demo.yaml"))
    # await fsm.to_IDLE()
    # trigger transition service
    while rclpy.ok():
        # await fsm.process_pending_transition()
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
    node.declare_parameter("package_name", "stingray_missions")
    fsm = FSM(node=node)
    event_loop = asyncio.get_event_loop()
    future = asyncio.wait(
        [ros_loop(node), fsm_loop(fsm)], return_when=asyncio.FIRST_EXCEPTION
    )
    done, _pending = event_loop.run_until_complete(future)
    for task in done:
        task.result()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
