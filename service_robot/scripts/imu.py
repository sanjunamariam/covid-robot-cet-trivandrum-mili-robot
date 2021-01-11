#!/usr/bin/env python
import math
from math import pi
import rospy
import FaBo9Axis_MPU9250
from tf.transformations import *
from sensor_msgs.msg import Imu
#using a library called FaBo9Axis_MPU9250 written for raspberry pi
mpu9250 = FaBo9Axis_MPU9250.MPU9250()
rospy.init_node('imu')

pub = rospy.Publisher('imu_data',Imu, queue_size = 1)

msg = Imu()
#msg.orientation_covariance[0] = -1
msg.header.frame_id = 'imu_link'
rate = rospy.Rate(100)

while not rospy.is_shutdown():
    accel = mpu9250.readAccel()
    gyro = mpu9250.readGyro()

    msg.header.stamp = rospy.Time.now()
    # mult by g - gravity compensation - linear acc is in g (datasheet).. need to convert to m/s2
    # https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Datasheet.pdf page 9
    # https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Datasheet.pdf page 8
    # At default sensitivity of 250deg/s we need to scale by 131.
    msg.linear_acceleration.x = accel['x']* (9.81)
    msg.linear_acceleration.y = accel['y']* (9.81)
    msg.linear_acceleration.z = accel['z']* (9.81)
    # mult by deg to rad value 0.01745 ( value in degree/s)
    # https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Datasheet.pdf page 8
    # https://github.com/jusgomen/ros-mpu9250-imu/blob/master/src/talker.cpp 0.070 sensitivity
    msg.angular_velocity.x = gyro['x'] * 0.01745329251 * 0.070
    msg.angular_velocity.y = gyro['y'] * 0.01745329251 * 0.070
    msg.angular_velocity.z = gyro['z'] * 0.01745329251 * 0.070

    msg.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    msg.angular_velocity_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    msg.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    #conversion of acceleration to ypr 
    #acceleration as gravity component (raw data have gravity component)
    #in rad/s
    #euler angle from accel
    roll = math.atan(-accel['x']/math.sqrt(pow(accel['y'],2)+pow(accel['z'],2)))
    pitch = math.atan(accel['y']/math.sqrt(pow(accel['x'],2)+pow(accel['z'],2)))
    #about z axis #
    #yaw = math.atan(accel['y']/accel['z']) 
    mag = mpu9250.readMagnet()
    Yh = (mag['y'] * cos(roll)) - (mag['z'] * sin(roll))
    Xh = (mag['x'] * cos(pitch)) + (mag['y'] * sin(roll) * sin(pitch)) + (mag['z'] * cos(roll) * sin(pitch))

    yaw = math.atan(Yh,Xh)
    #conversion of ypr to quaternion 
    #ypr in rad/s #sxyz first character : rotations are applied to ‘s’tatic or ‘r’otating frame - verify
    quat = quaternion_from_euler(roll, pitch, yaw, 'sxyz')

    #cy = math.cos(yaw * 0.5);
    #sy = math.sin(yaw * 0.5);
    #cp = math.cos(pitch * 0.5);
    #sp = math.sin(pitch * 0.5);
    #cr = math.cos(roll * 0.5);
    #sr = math.sin(roll * 0.5);

    #qw = cr * cp * cy + sr * sp * sy;
    #qx = sr * cp * cy - cr * sp * sy;
    #qy = cr * sp * cy + sr * cp * sy;
    #qz = cr * cp * sy - sr * sp * cy;

    msg.orientation.x = quat[0]; 
    msg.orientation.y = quat[1];
    msg.orientation.z = quat[2];
    msg.orientation.w = quat[3];
    #covariance matrix - unknown - all zero 
    #publish topic
    pub.publish(msg)
    rate.sleep()
