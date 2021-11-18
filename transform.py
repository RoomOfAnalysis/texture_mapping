import numpy as np
import math

A = np.array([[ 0.99903, 0.03822, -0.02210, 0.01095],
              [-0.03472, 0.98934,  0.14145, 0.10689],
              [-0.02727, 0.14054, -0.98970, 0.16761],
              [ 0.00000, 0.00000,  0.00000, 1.00000]])

print(A[:3, :3])

# Rot from A is orthogonal matrix (Rt = Ri)

print("AI")
print(np.linalg.inv(A))

print("AT")
print(A.T)
              
R = np.linalg.inv(A[:3, :3])  #A[:3, :3].T
T = A[:3, 3]

print("Rot from AT:")
print(R)

print("Trans from AT:")
print(T)

# inverse z
T *= np.array([1, 1, -1])
print("Trans:")
print(T)

Euler = np.array([8.132, 1.279, 357.990]) * np.pi / 180
# inverse direction
Euler *= np.array([-1, -1, 1])
print("Euler:")
print(Euler)

x_rot = np.array([[1, 0,                   0],
                  [0, math.cos(Euler[0]), -math.sin(Euler[0])],
                  [0, math.sin(Euler[0]),  math.cos(Euler[0])]])

y_rot = np.array([[ math.cos(Euler[1]), 0, math.sin(Euler[1])],
                  [ 0,                  1, 0],
                  [-math.sin(Euler[1]), 0, math.cos(Euler[1])]])

z_rot = np.array([[math.cos(Euler[2]), -math.sin(Euler[2]), 0],
                  [math.sin(Euler[2]),  math.cos(Euler[2]), 0],
                  [0,                   0,                  1]])

# https://en.wikibooks.org/wiki/Cg_Programming/Unity/Rotations#Moving_Rotation_Axes

# Z, X, Y => yaw-pitch-roll
# extrinsic rotations
print("ZXY: ")
print(np.matmul(y_rot, np.matmul(x_rot, z_rot)))

# Y, X, Z
# intrinsic rotations
# intrinsic rotations are equivalent to extrinsic rotations in reverse order

# print("YXZ: ")
# print(np.matmul(z_rot, np.matmul(x_rot, y_rot)))

# b_rot = np.array([[1,  0,  0],
#                   [0, -1,  0],
#                   [0,  0, -1]])
# print("R: ")
# print(np.matmul(z_rot, np.matmul(x_rot, np.matmul(y_rot, b_rot))))

R[0, 2] *= -1
R[1, :] *= -1
R[2, 0] *= -1
print("R from Rot:")
print(R)