# VJM MODEL
Making a VJM model for [tripetron robot](https://www.youtube.com/watch?v=beuY401hfh8) <br/>
The goal is to create deflection maps for 100N force in X, Y, Z directions<br/>
This is implemented in **showDef_lin()** which takes a force as a parameter and return deflection
### My approach
I started by drawing the vjm model
[!image](link to the photo)

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