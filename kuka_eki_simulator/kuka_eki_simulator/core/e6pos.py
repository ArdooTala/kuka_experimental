"""E6Pos data class mirroring the KUKA E6POS type (Cartesian pose)."""

CARTESIAN_AXES = ("X", "Y", "Z", "A", "B", "C")


class E6Pos:
    """KUKA E6POS-like Cartesian pose: X, Y, Z in mm and A, B, C in degrees."""

    def __init__(self, x=0.0, y=0.0, z=0.0, a=0.0, b=0.0, c=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.a = float(a)
        self.b = float(b)
        self.c = float(c)

    def to_cartesian_xml(self, element):
        """Write X, Y, Z, A, B, C as attributes of an XML element."""
        for name, value in zip(CARTESIAN_AXES, (self.x, self.y, self.z, self.a, self.b, self.c)):
            element.set(name, f"{value:g}")

    def __str__(self):
        return (
            f"E6Pos(X={self.x:g}, Y={self.y:g}, Z={self.z:g}, "
            f"A={self.a:g}, B={self.b:g}, C={self.c:g})"
        )
