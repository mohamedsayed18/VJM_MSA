# VJM MODEL
Making a VJM model for [tripetron robot](https://www.youtube.com/watch?v=beuY401hfh8) <br/>
The goal is to create deflection maps for 100N force in X, Y, Z directions<br/>
This is implemented in **showDef_lin()** which takes a force as a parameter and return deflection
### My approach
I started by drawing the vjm model
![](https://github.com/mohamedsayed18/VJM_MSA/blob/master/vjm_model.jpg)

then i started to follow the equations required to solve the problem using top bottom approach which is more intuitive for me
* dt = K * F
* K = inv(kc1 + kc2)
* Kc1 = kc0 - kc0 * jq * kcq
* Kc0 = inv(jt * inv(kt) * jt')
* Kcq = inv(jq * inv(jt * Kt * j') * jq) * jq * inv(jt * inv(kt) * jt')
* jq = jacobian for passive joints 
* jt = jacobian for theta

### Code implementations 
The main function is the **showDef_lin(f)**
```python
def showDef_lin(f):
    """
    calculate the deflections dt
    :param f: the force
    :return: dt
    """
```
at first we create three list of possible x,y and z positions
```python
x = [range(-200,200,5)] 
y = [range(-200, 200, 5)]  
z = [range(-200, 200, 5)]  
```
then loop through these ranges to calculate the k for each configuration
```python
K1 = VJM_total(i,j,k)
```
finally calculate the deflection and calculate its magnitude
```python
dt = K1 @ f  # dt is a (6x1) matrix
# the magnitude of the deflection
dr = math.sqrt(dt[0]**2 + dt[1]**2 + dt[2]**2);
```
**VJM_total(x, y, z)**<br/>
this function takes the position of the end effector and return the stiffness values
```python
:param x: the x position of the end effector
:param y: the y position of the end effector
:param z: the z position of the end effector

:return: k: stiffness matrix
```
first it calculate the joint values given x,y and z
```python
q = inv_kin(x,y,z) # it return list of joint values
```
and then return the k
```python
k1 = inv(vjm_lin(Tbase,Ttool,q0,q,t,L,l,d))
```
**vjm_lin()**<br/>
this function calculates the K values following this formula<br/>
Kc0 = inv(jt * inv(kt) * jt')
Kc1 = kc0 - kc0 * jq * kcq <br/>
for calculating the inverse kinematics i created two function
**inv_RR(x, y)** which solve the inverse kinematics for RR robot then i build a wrap up function **inv_kin(x, y, z)** which calculate inverse kinematics for the whole robot<br/>
**inv_RR(x, y)**
```python
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
```
**inv_kin(x, y , z)**<br/>
calculate the joint values for all joints of the robot
```python
q1, q2 = inv_RR(x, -y)
q3, q4 = inv_RR(3-z, y)
q5, q6 = inv_RR(3-x, z)
return [q1, q2, q3, q4, q5, q6]
```

**Jq(Tbase, Ttool, q0, q, t, L, l, d)**<br/>
this function calculate Jq, first it calculates the homogeneous transform
```python
T0 = f_k(Tbase, Ttool, q0, q, t, L, l, d)
```
then our jacobian will be some elements of the transformation 
```python
J1 = np.array([Td[0, 3], Td[1, 3], Td[2, 3], Td[2, 1], Td[0, 2], Td[1, 0]])
```
**Jt(Tbase, Ttool, q0, q, t, L, l, d)**<br/>
this function calculate J_theta, first it calculates the homogeneous transform
```python
T0 = f_k(Tbase, Ttool, q0, q, t, L, l, d)
```
then calculate the jacobians very similar to the jq and aggregate them into one list
```python
Jt = [J1, J2, J3 J4 J5 J6 J7 J8 J9 J10 J11 J12 J13]
```
**f_k(Tbase, Ttool, q0, q, t, L, l, d)**<br/>
this function takes the Transformations and joint angles and homogeneous matrix by simple substitution in the form
```python
T = Tbase.dot(Tx(-d / 2)).dot(Rz(q0(1))).dot(Rz(t(1))).dot(Tx(L)).dot(Tx(t(2))).dot(Ty(t(3))).dot(Tz(t(4))).dot(Rx(t(5))
).dot(Rx(t(5))).dot(Ry(t(6))).dot(Rz(t(7))).dot(Rz(q(1))).dot(Tx(l)).dot(
Tx(t(8))).dot(Ty(t(9))).dot(Tz(t(10))).dot(Rx(t(11))).dot(Ry(t(12))).dot(Rz(t(13))).dot(Ttool)
```

# MSA for tripteron manipulator
## Goals
Develop MSA model
W = K * delta_t

calculate the Cartesian stiffness matrix 
Kc = D1 − C1 . A1(inv) . B1 <br/>
Create deflection maps for 100N force in X, Y, Z directions<br/>
We = K * delta_t @ We = 100, find delta_t

## Steps
* develop the MSA model
![](https://github.com/mohamedsayed18/VJM_MSA/blob/master/msa_model.jpg)
* write the equation for the model as follow<br/>
every equation is wrote like a list of pairs the first element represent the delta_t and second element is the element to multiply for example 
first equation is: 0 = Ar_star * d_t0 is written as this e1 = [(0, Ar_star)] <br/>
At the fixtures
```python
e1 = [(0, Ar_star)]
e2 = [(6, Ar_star)]
e3 = [(12, Ar_star)]
```
At the end effector
```python

e4 = [(5,Ar_star), (18,Ar_star)]
e5 = [(11,Ar_star), (18,Ar_star)]
e5 = [(17,Ar_star), (18,Ar_star)]
```
The deflections
```python
e6 = [(1,Ar_star), (2,-Ar_star)]
e7 = [(3,Ar_star), (4,-Ar_star)]
e8 = [(7,Ar_star), (8,-Ar_star)]
e9 = [(9,Ar_star), (10,-Ar_star)]
e10 = [(13,Ar_star), (14,-Ar_star)]
e11 = [(15,Ar_star), (16,-Ar_star)]
```
At wrenches
```python
eq12 = [(0,Ap.dot(k11)), (1,Ap.dot(k12))] #w0
eq13 = [(0,Ap.dot(k21)), (1,Ap.dot(k22))]
eq14 = [(2,Ap.dot(k11)), (3,Ap.dot(k12))] #w2
eq15 = [(2,Ap.dot(k21)), (3,Ap.dot(k22))]
eq16 = [(4,Ap.dot(k11)), (5,Ap.dot(k12))] #w4
eq15 = [(4,Ap.dot(k21)), (5,Ap.dot(k22))]
eq14 = [(6,Ap.dot(k11)), (7,Ap.dot(k12))]
eq15 = [(6,Ap.dot(k21)), (7,Ap.dot(k22))]
eq14 = [(8,Ap.dot(k11)), (9,Ap.dot(k12))]
eq15 = [(8,Ap.dot(k21)), (9,Ap.dot(k22))]
eq14 = [(10,Ap.dot(k11)), (11,Ap.dot(k12))]
eq15 = [(2,Ap.dot(k21)), (3,Ap.dot(k22))]
```
To put this equations togther i created to functions<br/>
**eq_into_matrix(eq)**<br/>
it creates one row of the of the matrix given the equation 
```python
#:param eq:equation of the row
#:return: m: np array of this row
for i in eq:
    m[:, i[0]*6:(i[0]*6)+6] = i[1]
```
**matrix_form(eq)**<br/>
this function take list of all equations and return the stiffness
```python
"""
stack single matrices vertically to create the stiffness matrix
:param eq: list of all the equation
:return: stiffness matrix
"""
w = np.empty((5, 114))
for i in eq:
    row = eq_into_matrix(i)
    w = np.vstack((w, row)) # stack every equation into the matrix
return w
```
**kc(m)**<br/>
this function takes the m matrix and return the kc matrix
```python
A = m[:-1,:-1]
B = m[:-1, -1]
C = m[-1,:-1]
D = m[-1,-1]
return D - C @ np.linalg.inv(A) @ B
```