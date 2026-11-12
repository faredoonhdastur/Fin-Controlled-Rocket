# some math helpers for flight.py. 
# Vectors are plain 3-tuples.

import math


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def length(a):
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


# running average
def blend(old, new, k):
    return tuple(o + k * (n - o) for o, n in zip(old, new))


def clamp(x, limit):
    return max(-limit, min(limit, x))


# rotates v by angle (radians) about a unit axis via Rodrigues' formula
def rotate(v, axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    k_x_v = cross(axis, v)
    k_dot_v = sum(a * b for a, b in zip(axis, v))
    return tuple(v[i] * c + k_x_v[i] * s + axis[i] * k_dot_v * (1 - c) for i in range(3))


# nose and blue say which sensor axis points that way, e.g. ("y", +1).
# returns the nose, blue and orange directions in sensor axes, for to_body.
def body_axes(nose, blue):
    unit = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}
    n = tuple(nose[1] * c for c in unit[nose[0]])
    b = tuple(blue[1] * c for c in unit[blue[0]])
    return n, b, cross(n, b)


# sensor reading {'x':.., 'y':.., 'z':..} -> (nose, blue, orange)
def to_body(d, axes):
    v = (d["x"], d["y"], d["z"])
    return tuple(sum(a * b for a, b in zip(axis, v)) for axis in axes)
