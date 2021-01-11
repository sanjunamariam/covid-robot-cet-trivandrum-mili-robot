#!/usr/bin/env python

'''
Change Log:
    1-8-2020    Rojin   Chaneg the PWM Sections
                        Update the PID constands
                        Added the value motor_rated_ang_vel = 7.5 for easy changing
                        Simplified the w_r and w_l equations
                        
Custom Variables: (Double click to higlight and search)
    motor_rated_ang_vel
    kp
    kd
    ki                
    ticks_for_full_wheel_rotation
    
'''
# Code for taking in the values for velocity and converting it to 
# left and right motor PWM

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import time
from math import sin, cos, pi


tick_left_encoder = 0
tick_right_encoder = 0

ticks_for_full_wheel_rotation = 1600
previous_left_tick_encoder = 0
previous_right_tick_encoder = 0
kp = 1
kd = 0.1
ki = 0.01


def left_fb_callback(data):
    global tick_left_encoder
    tick_left_encoder = data.data
  
def right_fb_callback(data):
    global tick_right_encoder
    tick_right_encoder = data.data

#publishes pwm for right, left and direction flag
def callback(data):
    #radius of wheel (rough value in m 4.5cm)
    wheel_rad = 0.1
    #distance between wheels 18cm
    dist_bw_wheels = 0.33

    v = data.linear.x
    theta = data.angular.z
    #unit m/s
    #rospy.loginfo(rospy.get_name() + ": SPEED LINEAR %s" % v )
    #unit rad/s
    #rospy.loginfo(rospy.get_name() + ": ORIENTATION %s" % theta )
    v_r = (v) + ((dist_bw_wheels * theta)/(2))
    v_l = (v) - ((dist_bw_wheels * theta)/(2))
    #rospy.loginfo("linear v right %s" % v_r )
    #rospy.loginfo("linear v left %s" % v_l )
    w_r = (v_r/wheel_rad)
    w_l = (v_l/wheel_rad)
    #unit rad/s #desired
    #rospy.loginfo("angular v right %s" % w_r )
    #rospy.loginfo("angular v left %s" % w_l ) 

    #rospy.loginfo("pos encoder ticks received left : %s" % tick_left_encoder )
    #rospy.loginfo("pos encoder ticks received right : %s" % tick_right_encoder )
    current_time = rospy.Time.now()
 
 
    # distance travelled in one tick 
    # perimeter of wheel / number of tick counts for one rotation
    distancePerCount = (2 * pi * wheel_rad)/ticks_for_full_wheel_rotation;
    diff_tick_left = tick_left_encoder - previous_left_tick_encoder;
    diff_tick_right = tick_right_encoder - previous_right_tick_encoder;
	#diff denotes the no of additional ticks from last one.. 
    #so diff * distance per count will give you distance from last..
    # bt we need velocity.. so distance / time difference
    diff_distance_left = diff_tick_left * distancePerCount;
    diff_distance_right = diff_tick_right  * distancePerCount;
    time_diff = (current_time-last_time).to_sec();
	#need to verify whether linear or angular
    feedback_wl = diff_distance_left/time_diff;
    feedback_wr = diff_distance_right/time_diff;
 
 
    #PID
    #error
    error_r = w_r - feedback_wr
    error_l = w_l - feedback_wl
    current_time = rospy.Time.now()
    time = (current_time-last_time).to_sec()
    error_r_integral = 0
    error_l_integral = 0
    previous_error_r = 0 
    previous_error_l = 0
    #integral of error
    error_r_integral += error_r * time
    error_l_integral += error_l * time
    #derivative of error
    error_r_derivative = (error_r - previous_error_r)/time
    error_l_derivative = (error_l - previous_error_l)/time

    previous_error_r = error_r
    previous_error_l = error_l

    pid_r = (kp * error_r) + (ki * error_r_integral) + (kd * error_r_derivative)
    pid_l = (kp * error_l) + (ki * error_l_integral) + (kd * error_l_derivative)
    
    #pid_r = w_r
    #pid_l = w_l
    #ARDUINO
    #guess 72 rev per min .. need to change  72 rev /60 sec =  1.2 rev/sec = 7.5 rad/sec... so 7.5 rad/sec --> 255
    # so eqn (angular vel / 7.5 ) * 255 is the pwm signal for each motor
    # 30/30sec = 1 rev/sec (right) = 6.283
    # 25/30 sec = 0.833 rev/sec (left) = 5.235
    pub_right = rospy.Publisher("pwm_right", Int32, queue_size=50 )
    pub_left = rospy.Publisher("pwm_left", Int32, queue_size=50) 
    direc_flag_l = rospy.Publisher("direction_left", Int32, queue_size=50)
    direc_flag_r = rospy.Publisher("direction_right", Int32, queue_size=50)
    
    motor_rated_ang_vel = 4.71 # w = 2*pi*n/60 n=45
    pwmRight = abs((w_r/motor_rated_ang_vel)*255)
    pwmLeft = abs((w_l/motor_rated_ang_vel)*255)
    if (pwmRight >= 255):
	pwmRight = 255
    elif (pwmRight < 5):
	pwmRight = 0
    elif (pwmRight < 60):
        pwmRight += 35

    if (pwmLeft >= 255):
	pwmLeft = 255
    elif (pwmLeft < 5):
	pwmLeft = 0
    elif (pwmLeft < 60):
        pwmLeft += 35

    if (v_r > 0):
          direction_flag_r = 1 #cw
    if (v_r < 0):
          direction_flag_r = 0 #ccw
    if (v_r == 0):
          direction_flag_r = 2
    
    if (v_l > 0):
          direction_flag_l = 1 #cw
    if (v_l < 0):
          direction_flag_l = 0 #ccw
    if (v_l == 0):
          direction_flag_l = 2
    
    # to arduino
    pub_right.publish(pwmRight)
    pub_left.publish(pwmLeft)
    direc_flag_l.publish(direction_flag_l)
    direc_flag_r.publish(direction_flag_r)
    
    #rospy.loginfo("PWM right %s" % pwmRight )
    #rospy.loginfo("direction flag left %s" % direction_flag_l )
    #rospy.loginfo("PWM left %s" % pwmLeft )
    #rospy.loginfo("direction flag right %s" % direction_flag_r )


def listener():
    rospy.init_node('vel_listener', anonymous=True)
    
    global last_time
    last_time = rospy.Time.now()
    
    rospy.Subscriber("left_encoder_count", Int32, left_fb_callback)
    rospy.Subscriber("right_encoder_count", Int32, right_fb_callback)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
