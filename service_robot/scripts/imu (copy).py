#!/usr/bin/env python
import math
from math import pi
import rospy
import FaBo9Axis_MPU9250
from sensor_msgs.msg import Imu
#using a library called FaBo9Axis_MPU9250 written for raspberry pi
mpu9250 = FaBo9Axis_MPU9250.MPU9250()
rospy.init_node('imu')

pub = rospy.Publisher('imu_data',Imu, queue_size = 1)

msg = Imu()
msg.orientation_covariance[0] = -1
msg.header.frame_id = 'imu_link'
rate = rospy.Rate(100)

while not rospy.is_shutdown():
    accel = mpu9250.readAccel()
    gyro = mpu9250.readGyro()

    msg.header.stamp = rospy.Time.now()

    msg.linear_acceleration.x = accel['x']*9.81
    msg.linear_acceleration.y = accel['y']*9.81
    msg.linear_acceleration.z = accel['z']*9.81

    msg.angular_velocity.x = (gyro['x'] - 0.9)*0.01745329251
    msg.angular_velocity.y = (gyro['y'] - 0.5)*0.01745329251
    msg.angular_velocity.z = gyro['z']*0.01745329251

    #conversion of acceleration to ypr
    roll = math.atan(accel['y']/math.sqrt(pow(accel['x'],2)+pow(accel['z'],2))) * 180/pi
    pitch = math.atan(-1 * accel['x']/math.sqrt(pow(accel['y'],2)+pow(accel['z'],2))) * 180/pi 
    yaw = math.atan(accel['y']/accel['z']) * 180/pi 
    #conversion of ypr to quaternion 
    cy = math.cos(yaw * 0.5);
    sy = math.sin(yaw * 0.5);
    cp = math.cos(pitch * 0.5);
    sp = math.sin(pitch * 0.5);
    cr = math.cos(roll * 0.5);
    sr = math.sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;

    msg.orientation.x = qx; 
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    pub.publish(msg)
    rate.sleep()
