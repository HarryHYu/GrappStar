"""4x4 homogeneous transform helpers (numpy float64). Angles in degrees, lengths in mm."""
import math
import numpy as np

I4 = np.eye(4)


def T(x=0.0, y=0.0, z=0.0):
    """Translation. T((x, y, z)) also works."""
    if isinstance(x, (tuple, list, np.ndarray)):
        x, y, z = x
    M = np.eye(4)
    M[:3, 3] = (x, y, z)
    return M


def R(axis, deg, origin=None):
    """Rotation by deg about a unit axis through origin (default the frame origin). Right-hand rule."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    x, y, z = a
    C = 1 - c
    M = np.eye(4)
    M[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    if origin is not None:
        o = np.asarray(origin, float)
        M = T(o) @ M @ T(-o)
    return M


def Rx(deg):
    return R((1, 0, 0), deg)


def Ry(deg):
    return R((0, 1, 0), deg)


def Rz(deg):
    return R((0, 0, 1), deg)


def mul(*Ms):
    out = np.eye(4)
    for M in Ms:
        out = out @ M
    return out


def frame(origin, z_axis, x_hint=(1, 0, 0)):
    """Frame whose +Z points along z_axis, located at origin. Maps local -> parent."""
    z = np.asarray(z_axis, float); z = z / np.linalg.norm(z)
    xh = np.asarray(x_hint, float)
    if abs(np.dot(xh, z)) > 0.99:
        xh = np.array([0.0, 1.0, 0.0]) if abs(z[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = xh - np.dot(xh, z) * z; x /= np.linalg.norm(x)
    y = np.cross(z, x)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, origin
    return M


def z_to_x():
    """Rotate a +Z-axis solid so its axis lies along +X."""
    return Ry(90)


def z_to_y():
    """Rotate a +Z-axis solid so its axis lies along +Y."""
    return Rx(-90)


def apply(M, pts):
    """Apply 4x4 to an (N, 3) array of points."""
    p = np.asarray(pts, float)
    return p @ M[:3, :3].T + M[:3, 3]


def inv(M):
    Mi = np.eye(4)
    Rm = M[:3, :3]
    Mi[:3, :3] = Rm.T
    Mi[:3, 3] = -Rm.T @ M[:3, 3]
    return Mi


def to_m3(M):
    """4x4 -> the 3x4 nested list manifold3d.transform expects."""
    return np.asarray(M, np.float64)[:3, :]
