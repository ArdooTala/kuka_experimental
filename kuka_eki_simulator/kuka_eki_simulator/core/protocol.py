"""Builders for the KRC-side EKI XML frames (ProgramState and RobotState)."""

import xml.etree.ElementTree as ET

ROBOT_AXES = ("A1", "A2", "A3", "A4", "A5", "A6")
EXT_AXES = ("E1", "E2", "E3", "E4", "E5", "E6")
CARTESIAN_AXES = ("X", "Y", "Z", "A", "B", "C")


def prg_state_xml(status, stopped=False):
    """Build a <ProgramState> frame reporting the program status.

    0 idle, 1 executing, 5 stopped.
    """
    root = ET.Element("ProgramState")
    state = ET.SubElement(root, "State")
    state.set("Status", str(int(status)))
    state.set("Stopped", "true" if stopped else "false")
    return ET.tostring(root, short_empty_elements=False).decode("utf-8")


def cmd_state_xml(command_id, status=1):
    """Build a <ProgramState> frame reporting a command state (1 started, 2 finished, 3 error)."""
    root = ET.Element("ProgramState")
    command = ET.SubElement(root, "Command")
    command.set("Id", str(int(command_id)))
    state = ET.SubElement(command, "State")
    state.set("Status", str(int(status)))
    return ET.tostring(root, short_empty_elements=False).decode("utf-8")


def cmd_result_xml(command_id, code=0, message=""):
    """Build a <ProgramState> frame reporting a finished (code 0) or errored command."""
    root = ET.Element("ProgramState")
    command = ET.SubElement(root, "Command")
    command.set("Id", str(int(command_id)))
    state = ET.SubElement(command, "State")
    state.set("Status", "2" if code == 0 else "3")
    error = ET.SubElement(command, "Error")
    error.set("Code", str(int(code)))
    error.set("Message", message)
    return ET.tostring(root, short_empty_elements=False).decode("utf-8")


def robot_state_xml(robot):
    """Build a <RobotState> kinematics frame from a Robot (meta channel, one frame per client request)."""
    root = ET.Element("RobotState")
    position = ET.SubElement(root, "Position")

    joint = ET.SubElement(position, "Joint")
    for axis, value in zip(ROBOT_AXES, robot.joints.joint_array()):
        joint.set(axis, f"{float(value):g}")

    ext_joint = ET.SubElement(position, "ExtAxis")
    for axis, value in zip(EXT_AXES, robot.ext_joints.ext_array()):
        ext_joint.set(axis, f"{float(value):g}")

    cartesian = ET.SubElement(position, "Cartesian")
    cartesian_values = (
        robot.cartesian.x,
        robot.cartesian.y,
        robot.cartesian.z,
        robot.cartesian.a,
        robot.cartesian.b,
        robot.cartesian.c,
    )
    for axis, value in zip(CARTESIAN_AXES, cartesian_values):
        cartesian.set(axis, f"{float(value):g}")

    velocity = ET.SubElement(root, "Velocity")
    for axis, value in zip(ROBOT_AXES, robot.velocity):
        velocity.set(axis, f"{float(value):g}")

    torque = ET.SubElement(root, "Torque")
    for axis, value in zip(ROBOT_AXES, robot.torque):
        torque.set(axis, f"{float(value):g}")

    return ET.tostring(root, short_empty_elements=False).decode("utf-8")
