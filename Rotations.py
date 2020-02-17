import numpy as np


def Rx (q):
    rx = np.array([
        [1, 0, 0],
        [0, np.cos(q), np.sin(q)],
        [0, -np.sin(q), np.cos(q)]
    ])
    return rx


def Ry(q):
    ry = np.array([
        [np.cos(q), 0, -np.sin(q)],
        [0, 1, 0],
        [np.sin(q), 0, np.cos(q)]
    ])
    return ry


def Rz(q):
    rz = np.array([
        [0, 0, 1],
        [np.cos(q), np.sin(q), 0],
        [-np.sin(q), np.cos(q), 1]
    ])
    return rz

def Tx(d):
    t = np.array([
        [1, 0, 0, d],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return t

def Ty(d):
    t = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, d],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return t

def Tz(d):
    t = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])
    return t


print(np.identity(4))

