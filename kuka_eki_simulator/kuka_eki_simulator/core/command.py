"""Parsing of <RobotCommand> XML frames sent by the hardware interface."""

import logging
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

from kuka_eki_simulator.core.e6axis import E6Axis

logger = logging.getLogger(__name__)

# Strips leading whitespace and stray XML declarations, which appear when the
# driver sends several batches (each with its own prolog) close together and
# they end up in the same TCP frame.
_XML_LEAD_RE = re.compile(rb"^\s*(?:<\?xml[^>]*\?>\s*)*")


class CommandType:
    """RobotCommand Type attribute values (see rbt::CommandType)."""

    NONE = 0
    COMBINED = 1
    MOVE = 2
    GRIP = 3
    ABORT = 4
    CUSTOM = 5


class MoveMode:
    """Move Mode attribute values (see rbt::MoveMode)."""

    NONE = 0
    JOINT = 1
    CARTESIAN_PTP = 2
    CARTESIAN_LIN = 3
    TEACHED = 4
    CARTESIAN_CIRC = 6


@dataclass
class RobotCommand:
    """A parsed <RobotCommand> frame."""

    id: int
    type: int
    mode: int = MoveMode.NONE
    velocity: float = 0.0
    joint: E6Axis = field(default_factory=E6Axis)
    ext_joint: E6Axis = field(default_factory=E6Axis)
    abort: bool = False
    reset: bool = False


def parse_robot_command(data):
    """Parse one complete <RobotCommand> XML frame, or return None on failure."""
    try:
        if isinstance(data, str):
            data = data.encode("utf-8")
        data = _XML_LEAD_RE.sub(b"", data)
        root = ET.fromstring(data)
        if root.tag != "RobotCommand":
            logger.error("Unexpected root element: %s", root.tag)
            return None
        command = RobotCommand(id=int(root.attrib["Id"]), type=int(root.attrib["Type"]))
        if command.type == CommandType.ABORT:
            command.abort = root.find("AbortCommands") is not None
            command.reset = root.find("ResetAbortCommands") is not None
            return command
        move = root.find("Move")
        if move is not None:
            command.mode = int(move.attrib.get("Mode", MoveMode.NONE))
            command.velocity = float(move.attrib.get("Velocity", 0.0))
            command.joint = E6Axis.from_joint_element(move.find("Joint"))
            command.ext_joint = E6Axis.from_ext_element(move.find("ExtAxis"))
        return command
    except (ET.ParseError, ValueError, KeyError) as error:
        logger.error("Failed to parse RobotCommand: %s", error)
        return None
