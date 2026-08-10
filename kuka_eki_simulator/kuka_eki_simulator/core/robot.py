"""Simulated KUKA robot: joint state and synchronized constant-velocity PTP motions."""

import numpy as np

from kuka_eki_simulator.core.e6axis import E6Axis
from kuka_eki_simulator.core.e6pos import E6Pos

RAD2DEG = 180.0 / np.pi
DEFAULT_VELOCITY = 0.5  # rad/s


class Robot:
    """Holds the simulated joint state and executes PTP joint motions.

    Motions are synchronized constant-velocity point-to-point moves: all axes
    start and stop together, each axis moving at the travelled distance divided
    by the overall duration. No acceleration phase is modelled.
    """

    def __init__(self, initial_joints, initial_ext_joints=None):
        self.joints = initial_joints  # current E6Axis, degrees
        zero_ext = E6Axis(e1=0.0, e2=0.0, e3=0.0, e4=0.0, e5=0.0, e6=0.0)
        self.ext_joints = initial_ext_joints or zero_ext
        self.cartesian = E6Pos()
        self.velocity = np.zeros(6)  # deg/s
        self.torque = np.zeros(6)
        self._start = np.zeros(6)
        self._delta = np.zeros(6)
        self._ext_start = None
        self._ext_delta = None
        self._duration = 0.0
        self._elapsed = 0.0
        self._velocity_deg = np.zeros(6)
        self._ext_velocity_deg = np.zeros(6)
        self._moving = False

    def start_ptp(self, target, target_ext=None, velocity_rad_s=0.0, move_time=0.0):
        """Start a PTP move to ``target`` joint angles in degrees.

        NaN target axes keep their current value. Returns True if the move is
        instantaneous (no axis moved).
        """
        self._start = self.joints.joint_array()
        target_values = target.joint_array()
        self._delta = np.where(np.isnan(target_values), 0.0, target_values - self._start)

        self._ext_start = None
        self._ext_delta = None
        if target_ext is not None:
            self._ext_start = self.ext_joints.ext_array()
            ext_target = target_ext.ext_array()
            self._ext_delta = np.where(np.isnan(ext_target), 0.0, ext_target - self._ext_start)

        speed = velocity_rad_s * RAD2DEG
        if not np.isfinite(speed) or speed <= 0.0:
            speed = DEFAULT_VELOCITY * RAD2DEG

        distances = np.abs(self._delta)
        if self._ext_delta is not None:
            distances = np.concatenate((distances, np.abs(self._ext_delta)))
        max_distance = float(np.max(distances)) if distances.size else 0.0

        if max_distance <= 1e-9:
            duration = 0.0
        else:
            duration = max_distance / speed
        if move_time > 0.0 and move_time < duration:
            duration = move_time

        self._duration = duration
        self._elapsed = 0.0
        if duration > 1e-9:
            self._velocity_deg = self._delta / duration
            if self._ext_delta is not None:
                self._ext_velocity_deg = self._ext_delta / duration
            self._moving = True
        else:
            self._velocity_deg = np.zeros(6)
            self._ext_velocity_deg = np.zeros(6)
            self._moving = False
            self._snap_to(self._delta, self._ext_delta)
        return not self._moving

    def update(self, dt):
        """Advance the current motion by ``dt`` seconds."""
        if not self._moving:
            return
        self._elapsed += dt
        if self._elapsed >= self._duration:
            self._snap_to(self._delta, self._ext_delta)
            self._moving = False
            self.velocity = np.zeros(6)
        else:
            fraction = self._elapsed / self._duration
            self.joints = E6Axis(*(self._start + self._delta * fraction))
            if self._ext_start is not None and self._ext_delta is not None:
                self.ext_joints = self._ext_axis_from(self._ext_start + self._ext_delta * fraction)
            self.velocity = self._velocity_deg.copy()

    def is_moving(self):
        """Return True while a PTP motion is in progress."""
        return self._moving

    def stop(self):
        """Halt immediately at the current interpolated position."""
        if self._moving:
            fraction = min(1.0, self._elapsed / self._duration) if self._duration > 0.0 else 1.0
            ext_fraction = self._ext_delta * fraction if self._ext_delta is not None else None
            self._snap_to(self._delta * fraction, ext_fraction)
        self._moving = False
        self.velocity = np.zeros(6)

    def _snap_to(self, delta, ext_delta):
        self.joints = E6Axis(*(self._start + delta))
        if self._ext_start is not None and ext_delta is not None:
            self.ext_joints = self._ext_axis_from(self._ext_start + ext_delta)
        self.velocity = np.zeros(6)

    @staticmethod
    def _ext_axis_from(values):
        return E6Axis(
            e1=values[0],
            e2=values[1],
            e3=values[2],
            e4=values[3],
            e5=values[4],
            e6=values[5],
        )
