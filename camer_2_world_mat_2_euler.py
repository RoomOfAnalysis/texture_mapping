import math

camera2world = [
    [0.92157, 0.14839, 0.35874, 0.49447],
    [0.00582, 0.91868, -0.39497, 0.60627],
    [0.38817, -0.36608, -0.84576, -0.61212],
    [0.00000, 0.00000, 0.00000, 1.00000]
]

# camera2world = [
#     [0.34242, 0.28245, 0.89609, 1.88835],
#     [-0.00662, 0.95445, -0.29831, 1.01825],
#     [0.93952, -0.09622, -0.32869, -1.51790],
#     [0.00000, 0.00000, 0.00000, 1.00000]
# ]

# camera2world = [
#     [0.99903, 0.03822, -0.02210, 0.01095],
#     [-0.03472, 0.98934, 0.14145, 0.10689],
#     [-0.02727, 0.14054, -0.98970, 0.16761],
#     [0.00000, 0.00000, 0.00000, 1.00000]
# ]

# YXZ: https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix

a = math.atan(camera2world[0][2] / camera2world[2][2])
b = math.atan(-camera2world[1][2] / math.sqrt(1 - camera2world[1][2] * camera2world[1][2]))
c = math.atan(camera2world[1][0] / camera2world[1][1])

y = math.degrees(a)
x = math.degrees(b)
z = math.degrees(c)

#print(x, y, z)

y = y if y > 0 else 360 + y
x = -x if x < 0 else 360 - x
z = z if z > 0 else 360 + z

print(x, y, z)
