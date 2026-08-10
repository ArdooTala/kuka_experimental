"""E6Axis data class mirroring the KUKA E6AXIS type (joint values in degrees)."""

from math import nan

import numpy as np

JOINT_AXES = ("A1", "A2", "A3", "A4", "A5", "A6")
EXT_AXES = ("E1", "E2", "E3", "E4", "E5", "E6")


class E6Axis:
    """KUKA E6AXIS-like joint data: A1..A6 robot axes and E1..E6 external axes.

    All values are in degrees. Unset values default to NaN, meaning "not
    specified" in an incoming command.
    """

    def __init__(
        self,
        a1=nan,
        a2=nan,
        a3=nan,
        a4=nan,
        a5=nan,
        a6=nan,
        e1=nan,
        e2=nan,
        e3=nan,
        e4=nan,
        e5=nan,
        e6=nan,
    ):
        self.a1 = float(a1)
        self.a2 = float(a2)
        self.a3 = float(a3)
        self.a4 = float(a4)
        self.a5 = float(a5)
        self.a6 = float(a6)
        self.e1 = float(e1)
        self.e2 = float(e2)
        self.e3 = float(e3)
        self.e4 = float(e4)
        self.e5 = float(e5)
        self.e6 = float(e6)

    def joint_array(self):
        """Return A1..A6 as a numpy array."""
        return np.array([self.a1, self.a2, self.a3, self.a4, self.a5, self.a6])

    def ext_array(self):
        """Return E1..E6 as a numpy array."""
        return np.array([self.e1, self.e2, self.e3, self.e4, self.e5, self.e6])

    @classmethod
    def from_joint_element(cls, element):
        """Create an E6Axis from a <Joint> XML element (missing attributes become NaN)."""
        axis = cls()
        if element is not None:
            for name in JOINT_AXES:
                value = element.get(name)
                if value is not None:
                    setattr(axis, name.lower(), float(value))
        return axis

    @classmethod
    def from_ext_element(cls, element):
        """Create an E6Axis from an <ExtAxis> XML element (missing attributes become NaN)."""
        axis = cls()
        if element is not None:
            for name in EXT_AXES:
                value = element.get(name)
                if value is not None:
                    setattr(axis, name.lower(), float(value))
        return axis

    def to_joint_xml(self, element):
        """Write A1..A6 as attributes of an XML element."""
        for name, value in zip(JOINT_AXES, self.joint_array()):
            element.set(name, f"{value:g}")

    def to_ext_xml(self, element):
        """Write E1..E6 as attributes of an XML element."""
        for name, value in zip(EXT_AXES, self.ext_array()):
            element.set(name, f"{value:g}")

    def __str__(self):
        return (
            f"E6Axis(A1={self.a1:g}, A2={self.a2:g}, A3={self.a3:g}, "
            f"A4={self.a4:g}, A5={self.a5:g}, A6={self.a6:g})"
        )
