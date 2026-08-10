"""Command state machine mirroring the KRL program krl/motion_eki.src."""

import logging

from kuka_eki_simulator.core.command import CommandType, MoveMode
from kuka_eki_simulator.core.protocol import cmd_result_xml, cmd_state_xml, prg_state_xml

logger = logging.getLogger(__name__)


class MotionInterpreter:
    """Executes parsed RobotCommands against a Robot and reports ProgramState frames.

    Mirrors the behaviour of motion_eki.src: one command at a time with
    started/finished status reports, an idle report once the queue drains, and
    abort/reset handling via <AbortCommands/> / <ResetAbortCommands/>.
    """

    def __init__(self, robot):
        self.robot = robot
        self.pending = []
        self.stopped = False
        self._active = None
        self._idle_reported = True

    def enqueue(self, command):
        """Queue a parsed command for execution."""
        self.pending.append(command)

    def tick(self, dt):
        """Advance the simulation and return the ProgramState XML frames to send."""
        self.robot.update(dt)
        frames = []
        self._handle_abort_and_reset(frames)
        if self.stopped:
            return frames
        if self._active is not None and not self.robot.is_moving():
            frames.append(cmd_result_xml(self._active.id, code=0, message=""))
            self._active = None
            self._idle_reported = False
        if self._active is None and self.pending:
            command = self.pending.pop(0)
            frames.append(prg_state_xml(1, stopped=False))
            frames.append(cmd_state_xml(command.id, status=1))
            self._idle_reported = False
            self._execute(command, frames)
        if self._active is None and not self.pending and not self._idle_reported:
            frames.append(prg_state_xml(0, stopped=False))
            self._idle_reported = True
        return frames

    def _handle_abort_and_reset(self, frames):
        """Consume abort/reset commands; an abort clears the whole command buffer."""
        processed = []
        for command in self.pending:
            if command.type == CommandType.ABORT and command.abort:
                if not self.stopped:
                    self.stopped = True
                    self.robot.stop()
                    self._active = None
                    frames.append(prg_state_xml(5, stopped=True))
                    self._idle_reported = True
                # eki_clearbuffer: the abort and all remaining commands are dropped
                self.pending = []
                return
            if command.type == CommandType.ABORT and command.reset:
                if self.stopped:
                    self.stopped = False
                    frames.append(prg_state_xml(0, stopped=False))
                    self._idle_reported = True
                continue
            if self.stopped:
                continue  # moves are rejected while stopped
            processed.append(command)
        self.pending = processed

    def _execute(self, command, frames):
        """Dispatch one command, mirroring the KRL main loop."""
        if command.type == CommandType.MOVE:
            if command.mode == MoveMode.JOINT:
                finished = self.robot.start_ptp(
                    command.joint,
                    command.ext_joint,
                    velocity_rad_s=command.velocity,
                )
                if finished:
                    frames.append(cmd_result_xml(command.id, code=0, message=""))
                else:
                    self._active = command
            else:
                logger.warning("Unsupported move mode %d (id %d)", command.mode, command.id)
                message = "Cartesian motion not supported"
                frames.append(cmd_result_xml(command.id, code=1, message=message))
        elif command.type == CommandType.CUSTOM:
            frames.append(cmd_result_xml(command.id, code=0, message=""))
        else:
            # GRIP, COMBINED and unknown commands are consumed without effect
            frames.append(cmd_result_xml(command.id, code=0, message=""))
