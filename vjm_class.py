import math
import numpy as np
from numpy.linalg import inv
from Rotations import *


def showDef_lin(f):
    """
    calculate the deflections dt
    :param f: the force
    :return: dt
    """
    # generate range of x values
    x = [range(-200, 200, 5)] # todo we need to choose the range of our robot
    y = [range(-200, 200, 5)]  # todo we need to choose the range of our robot
    z = [range(-200, 200, 5)]  # todo we need to choose the range of our robot

    deflections = [] # to store the deflection values
    # loop to calculate the deflection for different values
    # todo may do triple loop to get the z
    for i in x:
        for j in y:
            for k in z:
                # calculate the robot stiffens
                K1 = VJM_total(i, j, k)
                # Get deflections for all configurations
                dt = K1 @ f  # dt is a (6x1) matrix
                # the magnitude of the deflection
                dr = math.sqrt(dt[0]**2 + dt[1]**2 + dt[2]**2);
                # store all the deflections
                deflections.append(dr);


def VJM_total(x, y, z):
    """

    :param x: the x position of the end effector
    :param y: the y position of the end effector
    :param z: the z position of the end effector

    :return: k: tuple of stiffness values for different configuration
    """
    #robot params
    L = 300e-3;
    l = 100e-3;
    d = 300e-3;

    # todo why HOWTO return Q: 4X4
    # todo and what values it represent
    q = inv_kin(x,y,z) # it return list of joint values
    Tbase = np.identity(4)
    Ttool = np.identity(4)
    t = np.zeros((1,13))

    q0, q1, q2, q3, q4, q5 = q

    # todo why negative -q1, -q2
    q = [q1, q2, q3]

    # todo what is the conditions before assigning
    # the k? i think it check if the inverse solution is valid
    k1 = inv(vjm_lin(Tbase,Ttool,q0,q,t,L,l,d) * 2 )
    return k1

def inv_RR(x, y):
    l1, l2 = 1, 1  # todo define globally
    p = math.sqrt(x**2+y**2)
    d = (-l2**2 + l1**2 + p**2)
    n = math.sqrt(l1**2 - d**2)

    phi = math.atan(y/x)
    alpha = math.atan(n/p-d)
    beta = math.atan(n/d)

    q1 = phi + beta
    q2 = -(alpha + beta)
    return q1, q2


def inv_kin(x, y, z):

    #calculate the joint values of the robot
    #:return q: list of joint values

    q1, q2 = inv_RR(x, -y)
    q3, q4 = inv_RR(3-z, y)
    q5, q6 = inv_RR(3-x, z)
    return [q1, q2, q3, q4, q5, q6]

def f_k(Tbase, Ttool, q0, q, t, L, l, d):
    T = Tbase.dot(Tx(-d / 2)).dot(Rz(q0(1))).dot(Rz(t(1))).dot(Tx(L)).dot(Tx(t(2))).dot(Ty(t(3))).dot(Tz(t(4))).dot(Rx(t(5))
    ).dot(Rx(t(5))).dot(Ry(t(6))).dot(Rz(t(7))).dot(Rz(q(1))).dot(Tx(l)).dot(
    Tx(t(8))).dot(Ty(t(9))).dot(Tz(t(10))).dot(Rx(t(11))).dot(Ry(t(12))).dot(Rz(t(13))).dot(Ttool)
    return T

def Jq (Tbase, Ttool, q0, q, t, L, l, d):
    T0 = f_k(Tbase, Ttool, q0, q, t, L, l, d)
    T0[:3, 3] = [0,0,0]
    T0 = np.transpose(T0)

    Td = Tbase @ Tx(-d / 2) @ Rz(q0(1)) @ Rz(t(1)) @ Tx(L) @ Tx(t(2)) @ Ty(t(3)) @ Tz(t(4)) \
         @ Rx(t(5)) @ Ry(t(6)) @ Rz(t(7)) * Rz(q(1)) * Tx(l) \
    @Tx(t(8)) @ Ty(t(9)) @ Tz(t(10)) @ Rx(t(11)) @ Ry(t(12)) * Rz(t(13)) @ Ttool @ T0


    J1 = np.array([Td[0, 3], Td[1, 3], Td[2, 3], Td[2, 1], Td[0, 2], Td[1, 0]])

    return J1.transpose()


def Jt(Tbase, Ttool, q0, q, t, L, l, d):
    T0 = f_k(Tbase, Ttool, q0, q, t, L, l, d)
    T0[:3, 3] = [0,0,0]
    T0 = np.transpose(T0)

    Td = Tbase @ Tx(-d / 2) @ Rz(q0(1)) @ Rz(t(1)) @ Tx(L) @ Tx(t(2)) @ Ty(t(3)) @ Tz(t(4)) \
         @ Rx(t(5)) @ Ry(t(6)) @ Rz(t(7)) * Rz(q(1)) * Tx(l) \
    @Tx(t(8)) @ Ty(t(9)) @ Tz(t(10)) @ Rx(t(11)) @ Ry(t(12)) * Rz(t(13)) @ Ttool @ T0


    J1 = np.array([Td[0, 3], Td[1, 3], Td[2, 3], Td[2, 1], Td[0, 2], Td[1, 0]])

    J1 = J1.transpose()

    return [J1] *13

def vjm_lin(Tbase, Ttool, q0, q, t, L, l, d):
    #calculate the Kc"""
    k0 = 1e6; # Actuatorstif

    # material and shape
    E = 70e9; # Young's modulus
    G = 25.5e9; # shearmodulus
    d = 50e-3; #
    L = 1;

    # for cylinder
    S = math.pi * d ^ 2 / 4;
    Iy = math.pi * d ^ 4 / 64;
    Iz = math.pi * d ^ 4 / 64;
    J = Iy + Iz

    # creating k11, k22
    k11 = np.zeros((6, 6))
    k11[0, 0] = E * S / L
    k11[1, 1] = 12 * E * Iz / L ** 3
    k11[2, 2] = 12 * E * Iy / L ** 3
    k11[3, 3] = G * J / L
    k11[4, 4] = 4 * E * Iy / L
    k11[5, 5] = 4 * E * Iz / L
    k11[1, 5] = -6 * E * Iz / L ** 2
    k11[2, 4] = 6 * E * Iy / L ** 2
    k11[4, 2] = 4 * E * Iy / L
    k11[5, 1] = 4 * E * Iz / L

    k22 = np.copy(k11)

    a = np.concatenate((k0, np.zeros((6,12))), axis=1)
    b = np.concatenate((np.zeros((6, 1)), k11, np.zeros((6, 6))), axis=1)
    c = np.concatenate((np.zeros((6, 1)), np.zeros((6,6)), k22), axis=1)

    kt = np.vstack((a, b, c))
    #todo number of return for the jt function different from jq
    # check jq_1 and jt_1
    jq = f_k(Tbase, Ttool, q0, q, t, L, l, d)
    jt = f_k(Tbase, Ttool, q0, q, t, L, l, d)

    #Analytical solution
    Kc0 = inv(jt.dot(inv(kt)).dot(np.transpose(jt)))

    Kc = Kc0 - Kc0 @ jq @ inv(np.transpose(jq) @ Kc0 @ jq) @ np.transpose(jq) @ Kc0
    return Kc