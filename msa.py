import numpy as np
""" """
# Some constants
E, S, L, Iy, Iz, G, J = 1, 1, 1, 1, 1, 1, 1


# the stiffness matrix
K = np.zeros((114, 114))

# passive joint geometry
Ar = np.diag((1, 1, 1, 1, 0, 1))
Ar_star = np.delete(Ar, -2, 0)
Ap = (np.array([0,0,0,0,1,0])).reshape(1,-1)

# creating k11
k11 = np.zeros((6,6))
k11[0,0] = E*S/L
k11[1,1] = 12*E*Iz/L**3
k11[2,2] = 12*E*Iy/L**3
k11[3,3] = G*J/L
k11[4,4] = 4*E*Iy/L
k11[5,5] = 4*E*Iz/L
k11[1,5] = -6*E*Iz/L**2
k11[2,4] = 6*E*Iy/L**2
k11[4,2] = 4*E*Iy/L
k11[5,1] = 4*E*Iz/L

# the k matrices
k12 = np.copy(k11)
k21 = np.copy(k11)
k22 = np.copy(k11)

# Create one row
def eq_into_matrix(eq):
    """
    :param eq:
    :return: m: np array of this row
    """
    m = np.zeros((eq[0][1].shape[0], 114))  # array with zeros

    for i in eq:
        m[:, i[0]*6:(i[0]*6)+6] = i[1]

    return m

#at fixtures
e1 = [(0, Ar_star)]
e2 = [(6, Ar_star)]
e3 = [(12, Ar_star)]

#end effector
e4 = [(5,Ar_star), (18,Ar_star)]
e5 = [(11,Ar_star), (18,Ar_star)]
e5 = [(17,Ar_star), (18,Ar_star)]

# deflections
e6 = [(1,Ar_star), (2,-Ar_star)]
e7 = [(3,Ar_star), (4,-Ar_star)]
e8 = [(7,Ar_star), (8,-Ar_star)]
e9 = [(9,Ar_star), (10,-Ar_star)]
e10 = [(13,Ar_star), (14,-Ar_star)]
e11 = [(15,Ar_star), (16,-Ar_star)]

# Wrenches
eq12 = [(0,Ap.dot(k11)), (1,Ap.dot(k12))] #w0
eq13 = [(0,Ap.dot(k21)), (1,Ap.dot(k22))]
eq14 = [(2,Ap.dot(k11)), (3,Ap.dot(k12))] #w2
eq15 = [(2,Ap.dot(k21)), (3,Ap.dot(k22))]
eq16 = [(4,Ap.dot(k11)), (5,Ap.dot(k12))] #w4
eq17 = [(4,Ap.dot(k21)), (5,Ap.dot(k22))]
eq18 = [(6,Ap.dot(k11)), (7,Ap.dot(k12))]
eq19 = [(6,Ap.dot(k21)), (7,Ap.dot(k22))]
eq20 = [(8,Ap.dot(k11)), (9,Ap.dot(k12))]
eq21 = [(8,Ap.dot(k21)), (9,Ap.dot(k22))]
eq22 = [(10,Ap.dot(k11)), (11,Ap.dot(k12))]
eq23 = [(2,Ap.dot(k21)), (3,Ap.dot(k22))]

eq = [e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, eq12, eq13, eq14, eq15, eq16, eq17, eq18, eq19, eq20, eq21, eq22, eq23]


# create matrix from all equation
def matrix_form(eq):
    """
    stack single matrices vertically to create the stiffness matrix
    :param eq: list of all the equation
    :return:
    """
    w = np.empty((5, 114))
    for i in eq:
        row = eq_into_matrix(i)
        w = np.vstack((w, row))
    return w


def kc (m):
    """
    calculate the kc matrix
    :param m: stiffness matrix
    :return: kc: the kc matrix
    """
    A = m[:-1,:-1]
    B = m[:-1, -1]
    c = m[-1,:-1]
    D = m[-1,-1]
    #D1 − C1 . A1(inv) . B1
    return D - c @ np.linalg.inv(A) @ B

M = matrix_form(eq)
k = kc(M)

