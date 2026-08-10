#!/usr/bin/env python3

"""Entry point of the KUKA motion primitives EKI simulator."""

import argparse
import time

import rclpy
from std_msgs.msg import String

from kuka_eki_simulator.core.command import parse_robot_command
from kuka_eki_simulator.core.e6axis import E6Axis
from kuka_eki_simulator.core.eki_server import MetaServer, MotionServer
from kuka_eki_simulator.core.interpreter import MotionInterpreter
from kuka_eki_simulator.core.protocol import robot_state_xml
from kuka_eki_simulator.core.robot import Robot

CYCLE_TIME = 0.004  # seconds, motion channel update rate


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(
        description="KUKA EKI motion primitives simulation over TCP/UDP"
    )
    parser.add_argument(
        "--eki_hw_iface_ip",
        default="127.0.0.1",
        help="The IP address of the EKI control interface (default=127.0.0.1)",
    )
    parser.add_argument(
        "--eki_hw_iface_motion_port",
        default=54600,
        help="The port of the EKI motion channel (default=54600)",
    )
    parser.add_argument(
        "--eki_hw_iface_meta_port",
        default=54601,
        help="The port of the EKI meta channel (default=54601, 0 disables)",
    )
    parser.add_argument(
        "--sen",
        default="ImFree",
        help='Type attribute in EKI XML doc. E.g. <Sen Type:"ImFree">',
    )
    args, _ = parser.parse_known_args()

    node = rclpy.create_node("kuka_eki_motion_primitives_simulation")
    node.get_logger().info("Started 'kuka_eki_motion_primitives_simulation' node.")

    state_pub = node.create_publisher(String, "~/eki/state", 1)
    command_pub = node.create_publisher(String, "~/eki/command", 1)
    meta_pub = node.create_publisher(String, "~/eki/meta", 1)

    robot = Robot(
        initial_joints=E6Axis(a1=0.0, a2=-90.0, a3=90.0, a4=0.0, a5=90.0, a6=0.0),
        initial_ext_joints=E6Axis(e1=0.0, e2=0.0, e3=0.0, e4=0.0, e5=0.0, e6=0.0),
    )
    interpreter = MotionInterpreter(robot)

    motion_server = MotionServer(args.eki_hw_iface_ip, args.eki_hw_iface_motion_port)
    motion_server.start()
    meta_server = None
    if int(args.eki_hw_iface_meta_port) > 0:
        meta_server = MetaServer(args.eki_hw_iface_ip, args.eki_hw_iface_meta_port)
        meta_server.start()

    last = time.monotonic()
    try:
        while rclpy.ok():
            now = time.monotonic()
            dt = now - last
            last = now

            for frame in motion_server.receive():
                command = parse_robot_command(frame)
                if command is None:
                    continue
                node.get_logger().info(
                    f"Received command id={command.id} type={command.type} mode={command.mode}"
                )
                msg = String()
                msg.data = frame.decode("utf-8")
                command_pub.publish(msg)
                interpreter.enqueue(command)

            if meta_server is not None:
                for request in meta_server.receive_requests():
                    if request == MetaServer.UPDATE:
                        meta_frame = robot_state_xml(robot)
                        meta_server.send(meta_frame.encode("utf-8"))
                        msg = String()
                        msg.data = meta_frame
                        meta_pub.publish(msg)
                    elif request == MetaServer.DISCONNECT:
                        node.get_logger().info("Meta channel reset requested")
                    elif request == MetaServer.CLEAR:
                        node.get_logger().info("Meta channel clear requested")
                        meta_server.close()
                        meta_server = None

            for frame in interpreter.tick(dt):
                msg = String()
                msg.data = frame
                state_pub.publish(msg)
                motion_server.send(frame.encode("utf-8"))

            elapsed = time.monotonic() - now
            if elapsed < CYCLE_TIME:
                time.sleep(CYCLE_TIME - elapsed)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        motion_server.close()
        if meta_server is not None:
            meta_server.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
