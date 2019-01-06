#!/usr/bin/env python

from MPU6050 import MPU6050

import rospy
import numpy as np

rospy.init_node('mpu6050')

# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = int(rospy.get_param('~x_acc', 0))
y_accel_offset = int(rospy.get_param('~y_acc', 0))
z_accel_offset = int(rospy.get_param('~z_acc', 0))
x_gyro_offset = int(rospy.get_param('~x_gyro', 0))
y_gyro_offset = int(rospy.get_param('~y_gyro', 0))
z_gyro_offset = int(rospy.get_param('~z_gyro', 0))

enable_debug_output = int(rospy.get_param('~debug', True))

i2c_bus = int(rospy.get_param('~i2c_bus', 1))
device_address = int(rospy.get_param('~device_addr', 0x68))

# Calibration parameters
buffer_size = int(rospy.get_param('~buffer_size', 1000))
accel_deadzone = int(rospy.get_param('~accel_deadzone', 8))
gyro_deadzone = int(rospy.get_param('~gyro_deadzone', 1))

# Variables to keep track of the readings
ROT = np.zeros(3)
ACCEL = np.zeros(3)

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

def shutdown_cb():
    rospy.loginfo("Shutting down MPU6050")
    mpu.reset()
rospy.on_shutdown(shutdown_cb)

def mean_sensors():
    # Number of samples that will be discarded at the beginning
    n_discard = 100
    for measure in xrange(buffer_size + n_discard):
        # Read raw accel and gyro measurements
        rot = mpu.get_rotation()
        acc = mpu.get_acceleration()
        # Discard first "n_discard" measurement
        if i > n_discard:
            ROT += rot
            ACCEL += acc
        # Sleep during 2 ms to avoid duplicated measurements
        rospy.sleep(2/1000.)
    # Compute average
    return ROT / buffer_size, ACCEL / buffer_size

def calibration():
    global a_x_offset, a_y_offset, a_z_offset, g_offset
    a_x_offset = -ACCEL[0] / 8.
    a_y_offset = -ACCEL[1] / 8.
    a_z_offset = (16384 - ACCEL[2]) / 8.
    g_offset = -ROT / 4.

    params_calibrated = [False] * 6

    while all(params_calibrated):
        mpu.set_x_accel_offset(a_x_offset)
        mpu.set_y_accel_offset(a_y_offset)
        mpu.set_z_accel_offset(a_z_offset)
        mpu.set_x_gyro_offset(g_offset[0])
        mpu.set_y_gyro_offset(g_offset[1])
        mpu.set_z_gyro_offset(g_offset[2])

        mean_sensors()
        rospy.loginfo("...")

        if abs(ACCEL[0]) <= accel_deadzone:
            params_calibrated[0] = True
        else:
            a_x_offset = (a_x_offset - ACCEL[0]) / accel_deadzone

        if abs(ACCEL[1]) <= accel_deadzone:
            params_calibrated[1] = True
        else:
            a_y_offset = (a_y_offset - ACCEL[1]) / accel_deadzone

        if abs(16384 - ACCEL[2]) <= accel_deadzone:
            params_calibrated[2] = True
        else:
            a_z_offset = (a_z_offset - ACCEL[2]) / accel_deadzone

        if abs(ROT[0]) <= gyro_deadzone:
            params_calibrated[3] = True
        else:
            g_offset[0] = (g_offset[0] - ROT[0]) / gyro_deadzone

        if abs(ROT[1]) <= gyro_deadzone:
            params_calibrated[4] = True
        else:
            g_offset[1] = (g_offset[1] - ROT[1]) / gyro_deadzone

        if abs(ROT[2]) <= gyro_deadzone:
            params_calibrated[5] = True
        else:
            g_offset[2] = (g_offset[2] - ROT[2]) / gyro_deadzone

while not rospy.is_shutdown():
    rospy.loginfo("Reading sensor values...")
    mean_sensors()
    rospy.sleep(1)

    rospy.loginfo("Calculating offsets...")
    calibration()
    rospy.sleep(1)

    mean_sensors()
    rospy.loginfo("FINISHED!")
    rospy.loginfo("Sensor readings with offset:")
    rospy.loginfo(ACCEL[0])
    rospy.loginfo(ACCEL[1])
    rospy.loginfo(ACCEL[2])
    rospy.loginfo(ROT[0])
    rospy.loginfo(ROT[1])
    rospy.loginfo(ROT[2])
    rospy.loginfo("Your offsets:")
    rospy.logwarn("x_acc: {}".format(a_x_offset))
    rospy.logwarn("y_acc: {}".format(a_y_offset))
    rospy.logwarn("z_acc: {}".format(a_z_offset))
    rospy.logwarn("x_gyro: {}".format(g_offset[0]))
    rospy.logwarn("y_gyro: {}".format(g_offset[1]))
    rospy.logwarn("z_gyro: {}".format(g_offset[2]))

    rospy.loginfo("Replace these values into calibrated.yaml")

    exit()
