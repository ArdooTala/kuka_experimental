#!/usr/bin/env python3

import argparse
import sys
import socket
import numpy as np
import time
import xml.etree.ElementTree as ET
import errno
import rclpy
import threading
from std_msgs.msg import String

max_vel = 1.0 * 100.0

def create_eki_xml_rob(act_joint_pos, command_id="1", ext_ax_pos=None, in_motion=False):
    q = act_joint_pos
    qd = [100.0 if in_motion else 0.0] * 6  # Joint velocities
    eff = [0.0] * 6  # Joint torques

    if ext_ax_pos is None:
        ext_ax_pos = [0.0] * 6

    root = ET.Element('RobotState')

    # Command
    command = ET.SubElement(root, 'Command')
    if in_motion:
        command.set('Id', str(command_id))
        command.set('Finished_Id', str(command_id - 1))
    else:
        command.set('Id', "0")
        command.set('Finished_Id', str(command_id))

    command.set('Stopped', "0")

    # Joint positions
    position = ET.SubElement(root, 'Position')
    joint = ET.SubElement(position, 'Joint')
    for i in range(6):
        joint.set(f'A{i+1}', str(q[i]))
    #joint.set('A7', "0.0")  # Placeholder for A7

    ext_joint = ET.SubElement(position, 'ExtAxis')
    for i in range(6):
        ext_joint.set(f'E{i+1}', str(ext_ax_pos[i]))

    # Cartesian positions (placeholder values)
    cartesian = ET.SubElement(position, 'Cartesian')
    for axis in ['X', 'Y', 'Z', 'A', 'B', 'C']:
        cartesian.set(axis, "0.0")

    # Joint velocities
    velocity = ET.SubElement(root, 'Velocity')
    for i in range(6):
        velocity.set(f'A{i+1}', str(qd[i] / max_vel))

    # Joint torques
    torque = ET.SubElement(root, 'Torque')
    for i in range(6):
        torque.set(f'A{i+1}', str(eff[i]))

    # Gripper state (placeholders)
    gripper = ET.SubElement(root, 'Gripper')
    jaw = ET.SubElement(gripper, 'Jaw')
    jaw.set('Position', "0.0")
    jaw.set('Status', "0")
    vacuum = ET.SubElement(gripper, 'Vacuum')
    vacuum.set('Suction', "0")
    vacuum.set('Force1', "0.0")
    vacuum.set('Force2', "0.0")
    vacuum.set('Cylinder', "0")

    # Info (placeholder)
    info = ET.SubElement(root, 'Info')
    info.set('Code', "0")
    info.set('Message', "OK")

    # Error (placeholder)
    error = ET.SubElement(root, 'Error')
    error.set('Code', "0")
    error.set('Message', "")

    return ET.tostring(root, short_empty_elements=False)


def parse_eki_xml_sen(data):
    try:
        result = {}

        # Parse the XML data
        tree = ET.ElementTree(ET.fromstring(data))
        root = tree.getroot()

        # Extract command ID (from <RobotCommand Id="...">)
        command_id = root.attrib.get('Id')
        if command_id is None:
            raise ValueError("Missing 'Id' attribute in <RobotCommand> element")
        result['command_id'] = int(command_id)

        result['command_type'] = int(root.attrib.get('Type'))

        # Extract Mode from <Move> element
        move_element = root.find('.//Move')
        mode = int(move_element.attrib.get('Mode', -1))  # Default to -1 if Mode is missing

        # Extract joint values (from <Move><Joint A1="0.000000" A2="0.000000" ...>)
        joint = move_element.find('Joint')
        if joint is None:
            raise ValueError("Missing <Joint> element in <Move> section")

        joint_values = []
        for axis in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
            axis_value = joint.attrib.get(axis)
            if axis_value is None:
                raise ValueError(f"Missing joint value for {axis}")
            joint_values.append(float(axis_value))

        result['joint_positions'] = np.array(joint_values, dtype=np.float64)

        # Extract ext joint values (from <Move><Joint E1="0.000000" E2="0.000000" ...>)
        ext_joint = move_element.find('ExtAxis')
        if ext_joint is None:
            raise ValueError("Missing <ExtAxis> element in <Move> section")

        ext_joint_values = []
        for ext_axis in ['E1', 'E2', 'E3', 'E4', 'E5', 'E6']:
            ext_axis_value = ext_joint.attrib.get(ext_axis)
            if ext_axis_value is None:
                raise ValueError(f"Missing external joint value for {axis}")
            ext_joint_values.append(float(ext_axis_value))

        result['ext_joint_positions'] = np.array(ext_joint_values, dtype=np.float64)
        return result

    except Exception as e:
        print(f"[Error] Failed to parse RobotCommand: {e}")
        return None

def setup_and_accept(host, port, name, connections, conn_lock, node):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        node.get_logger().info(f"Successfully created TCP socket for {name} on ip:port {host}:{port}.")
        s.bind((host, port))
        s.listen(1)
        node.get_logger().info(f"Waiting for {name} connection.")
        conn, addr = s.accept()
        with conn_lock:
            connections[name] = {'conn': conn, 'addr': addr, 'sock': s}
        node.get_logger().info(f"{name} TCP connection established with {addr}.")
    except socket.error as e:
        node.get_logger().fatal(f"Could not setup socket for {name}. Error: {e}")
        raise e

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='KUKA EKI Simulation over TCP')
    parser.add_argument('--eki_hw_iface_ip', default="127.0.0.1", help='The IP address of the EKI control interface (default=127.0.0.1)')
    parser.add_argument('--eki_hw_iface_motion_port', default=54600, help='The port of the EKI control motion interface (default=54600)')
    parser.add_argument('--sen', default='ImFree', help='Type attribute in EKI XML doc. E.g. <Sen Type:"ImFree">')

    # Parse known arguments
    args, _ = parser.parse_known_args()
    host = args.eki_hw_iface_ip
    port_motion = int(args.eki_hw_iface_motion_port)
    sen_type = args.sen

    # Configuration
    node_name = 'kuka_eki_simulation_tcp'
    cycle_time = 0.004
    act_joint_pos = np.array([0, -90, 90, 0, 90, 0], dtype=np.float64)
    act_command_id = 0
    ext_ax_pos = None
    max_timeout = 5

    node = rclpy.create_node(node_name)

    node.get_logger().info(f"Started '{node_name}' node.")

    eki_act_pub = node.create_publisher(String, '~/eki/state', 1)
    eki_cmd_pub = node.create_publisher(String, '~/eki/command', 1)

    # Connections container
    connections = {}
    conn_lock = threading.Lock()

    # Accept incoming connection
    t_motion = threading.Thread(target=setup_and_accept, args=(host, port_motion, 'motion', connections, conn_lock, node))

    t_motion.start()

    t_motion.join()

    conn_motion = connections['motion']['conn']
    s_motion = connections['motion']['sock']

    conn_motion.settimeout(1)

    try:
        qu = []
        while rclpy.ok():
            time.sleep(0.001)  # FIXME: make this a ros2 node
            try:
                # Create and send robot state as XML
                str_data = create_eki_xml_rob(act_joint_pos, act_command_id, ext_ax_pos)
                msg = String()
                msg.data = str(str_data)
                eki_act_pub.publish(msg)
                conn_motion.send(str_data)  # Send data over TCP
                node.get_logger().debug(f"Sent XML:\n{str_data.decode('utf-8')}")

                # Receive the command message
                recv_msg = None
                try:
                    recv_msg = conn_motion.recv(8192)
                    if not recv_msg:
                        break  # No data received, close connection

                    node.get_logger().info(f"Received XML:\n---\n{recv_msg.decode('utf-8')}\n---\n")
                    msg = String()
                    msg.data = str(recv_msg)
                    eki_cmd_pub.publish(msg)

                    delim = b"</RobotCommand>"
                    data = [cm + delim for cm in recv_msg.split(delim) if cm.strip()]
                    qu += data
                except socket.timeout:
                    # continue
                    pass

                node.get_logger().info(f"Queue Len: {len(qu)}")
                if not qu:
                    continue

                recv_msg = qu.pop(0)
                # Parse the received XML and update the joint position and command ID
                parsed_data = parse_eki_xml_sen(recv_msg)
                node.get_logger().info(f"Parsed Data:\n{parsed_data}")

                if parsed_data is None:
                    continue

                act_command_id = parsed_data['command_id']
                ext_ax_pos = parsed_data['ext_joint_positions']

                if parsed_data['command_type'] == 2:
                    act_joint_pos = parsed_data['joint_positions']

                if parsed_data['command_type'] == 4:
                    node.get_logger().info(f"Executing Custom CMD")

                str_data = create_eki_xml_rob(act_joint_pos, act_command_id, ext_ax_pos, True)
                msg = String()
                msg.data = str(str_data)
                eki_act_pub.publish(msg)
                conn_motion.send(str_data)  # Send data over TCP
                node.get_logger().info(f"Sent XML:\n{str_data.decode('utf-8')}")

                time.sleep(cycle_time / 2)

            except socket.error as e:
                node.get_logger().error(f"Socket error: {e}")
                break

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        # Clean up and close the connection
        node.get_logger().info(f"Shutting down '{node_name}' node.")
        conn_motion.close()  # Close the TCP connection
        s_motion.close()  # Close the socket

    rclpy.shutdown()


if __name__ == '__main__':
    main()
