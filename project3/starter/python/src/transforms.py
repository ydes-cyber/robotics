"""Rigid 3D transforms; matrices are 4x4 rows internally."""
import math


def identity():
    return ((1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0))


def compose(a, b):
    return tuple(tuple(sum(a[row][k] * b[k][col] for k in range(4))
                       for col in range(4)) for row in range(4))


def from_rotation_translation(rotation, xyz):
    return tuple(tuple(rotation[row][col] for col in range(3)) + (xyz[row],)
                 for row in range(3)) + ((0.0, 0.0, 0.0, 1.0),)


def translation(xyz):
    return from_rotation_translation(identity(), xyz)


def rpy_rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                (-sp, cp * sr, cp * cr))
    return from_rotation_translation(rotation, (0.0, 0.0, 0.0))


def axis_angle(axis, angle):
    length = math.sqrt(sum(value * value for value in axis))
    if length == 0.0:
        return identity()  # zero movable axis is unspecified; avoid a crash
    x, y, z = (value / length for value in axis)
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    rotation = ((t*x*x+c, t*x*y-s*z, t*x*z+s*y),
                (t*x*y+s*z, t*y*y+c, t*y*z-s*x),
                (t*x*z-s*y, t*y*z+s*x, t*z*z+c))
    return from_rotation_translation(rotation, (0.0, 0.0, 0.0))


def quaternion_to_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if n == 0.0:
        return identity()
    x, y, z, w = x/n, y/n, z/n, w/n
    rotation = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    return from_rotation_translation(rotation, (0.0, 0.0, 0.0))


def matrix_to_quaternion(matrix):
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = 2.0 * math.sqrt(1.0 + trace)
        q = ((m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s,
             (m[1][0]-m[0][1])/s, s/4.0)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0]-m[1][1]-m[2][2])
        q = (s/4.0, (m[0][1]+m[1][0])/s, (m[0][2]+m[2][0])/s,
             (m[2][1]-m[1][2])/s)
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1]-m[0][0]-m[2][2])
        q = ((m[0][1]+m[1][0])/s, s/4.0, (m[1][2]+m[2][1])/s,
             (m[0][2]-m[2][0])/s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2]-m[0][0]-m[1][1])
        q = ((m[0][2]+m[2][0])/s, (m[1][2]+m[2][1])/s,
             s/4.0, (m[1][0]-m[0][1])/s)
    n = math.sqrt(sum(value*value for value in q))
    return tuple(value/n for value in q)


def flatten_column_major(matrix):
    return [matrix[row][col] for col in range(4) for row in range(4)]
