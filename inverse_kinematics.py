#!/usr/bin/env python
#from operator import matmul
import numpy as np
from numpy import matmul
np.set_printoptions(suppress=True)
from lab3_header import *
"""
Angles are in radian, distance is in meters.
"""
def Get_M():
    # =================== Your code starts here ====================#
    # Fill in the correct values for the M matrix
    M = np.eye(4)
    M[0,3] = -0.15
    M[1,3] = 0.15
    M = matmul(M, DHtoA(0,  -PI/2,  0,  0.162))
    M = matmul(M, DHtoA(-0.244,  0,     -PI,  0.120))
    M = matmul(M, DHtoA(-0.213,  0,     0,  -0.093))
    M = matmul(M, DHtoA(0,  -PI/2,  PI/2,  0.083))
    M = matmul(M, DHtoA(0,  PI/2,   0,  0.083))
    M = matmul(M, DHtoA(0.0535,  0,  0,  0.141))
# ==============================================================#
return M
def DHtoA(r, a, t, d):
# =================== Your code starts here ====================#
# Write a script for returning the transformation matrix for the link from 
the DH parameters
A = np.eye(4)
c  = np.cos([t, a])
s  = np.sin([t, a])
A[0,0] = c[0]
A[0,1] = -s[0]*c[1]
A[0,2] = s[0]*s[1]
A[0,3] = r*c[0]
A[1,0] = s[0]
A[1,1] = c[0]*c[1]
A[1,2] = -c[0]*s[1]
A[1,3] = r*s[0]
A[2,1] = s[1]
A[2,2] = c[1]
A[2,3] = d
# ==============================================================#
return A
"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
# Initialize the return_value 
return_value = [None, None, None, None, None, None]
# =========== Implement joint angle to encoder expressions here ===========
print("Foward kinematics calculated:\n")
# =================== Your code starts here ====================#
theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
T = np.eye(4)
M = Get_M()
#offset between world frame and 0th frame
T[0,3] = -0.15
T[1,3] = 0.15
#DH table
T = matmul(T, DHtoA(0,  -PI/2,  theta[0],  0.162))
T = matmul(T, DHtoA(-0.244,  0,     theta[1]-PI,  0.120))
T = matmul(T, DHtoA(-0.213,  0,     theta[2],  -0.093))
T = matmul(T, DHtoA(0,  -PI/2,  theta[3]+PI/2,  0.083))
T = matmul(T, DHtoA(0,  PI/2,   theta[4],  0.083))
T = matmul(T, DHtoA(0.0535,  0,  theta[5],  0.141))
# ==============================================================#
print(str(T) + "\n")
return_value[0] = theta1 + PI
return_value[1] = theta2
return_value[2] = theta3
return_value[3] = theta4 - (0.5*PI)
return_value[4] = theta5
return_value[5] = theta6
return return_value
