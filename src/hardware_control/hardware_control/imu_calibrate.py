from FaBo9Axis_MPU9250 import MPU9250
import time

imu = MPU9250()
avg_gyro_x = 0
avg_gyro_y = 0
avg_gyro_z = 0

for i in range(1000):
    gyro = imu.readGyro()
    avg_gyro_x += gyro["x"]
    avg_gyro_y += gyro["y"]
    avg_gyro_z += gyro["z"]

x_offset = avg_gyro_x / 1000
y_offset = avg_gyro_y / 1000
z_offset = avg_gyro_z / 1000

gyro = imu.readGyro()


# subtract from gyro readings
print("Gyro offsets:")
# print(gyro["x"])
print("x: " + str(x_offset))
# print(gyro["x"] - x_offset)
# print(gyro["y"])
print("y: " + str(y_offset))
# print(gyro["y"] - y_offset)
# print(gyro["z"])
print("z: " + str(z_offset))
# print(gyro["z"] - z_offset)

