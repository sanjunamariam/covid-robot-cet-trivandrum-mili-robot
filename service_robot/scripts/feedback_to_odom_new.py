#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import tf
import math
from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

tick_left_encoder = 0
tick_right_encoder = 0 
def callback(data):
    #listen to imu topic
    vx = data.angular_velocity.x
    vy = data.angular_velocity.y
    vz = data.angular_velocity.z

    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z

    orientation_x = data.orientation.x
    orientation_y = data.orientation.y
    orientation_z = data.orientation.z
    orientation_w = data.orientation.w

    #rospy.loginfo("orientation_w %s" % orientation_w )

def left_fb_callback(data):
    global tick_left_encoder
    tick_left_encoder = data.data 
    
def right_fb_callback(data):
    global tick_right_encoder
    tick_right_encoder = data.data

def listener():
    rospy.init_node('fb_to_odom_node', anonymous=True)
    last_time = rospy.Time.now()
    odometry_pub = rospy.Publisher("odom",Odometry, queue_size=50)
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=50)
    rospy.Subscriber("imu", Imu, callback)
    rospy.Subscriber("left_encoder_count", Int32, left_fb_callback)
    rospy.Subscriber("right_encoder_count", Int32, right_fb_callback)
    odom_broadcaster = tf.TransformBroadcaster()
    #radius of wheel (rough value in m 10cm)
    wheel_rad = 0.1
    #distance between wheels 47cm
    dist_bw_wheels = 0.47

    previous_left_tick_encoder = 0
    previous_right_tick_encoder = 0
    actual_x = 0
    actual_y = 0
    actual_theta = 0
############################need to modify #############################
    ticks_for_full_wheel_rotation = 1600

    
#############################
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
            
            #rospy.loginfo("pos encoder ticks received left : %s" % tick_left_encoder )
            #rospy.loginfo("pos encoder ticks received right : %s" % tick_right_encoder )
            current_time = rospy.Time.now()

	    # distance travelled in one tick 
	    #perimeter of wheel / number of tick counts for one rotation
	    distancePerTick = (2 * pi * wheel_rad)/ticks_for_full_wheel_rotation
	    diff_tick_left = tick_left_encoder - previous_left_tick_encoder
	    diff_tick_right = tick_right_encoder - previous_right_tick_encoder

	    #diff denotes the no of additional ticks from last one.. 
	    #so diff * distance per count will give you distance from last..
	    # bt we need velocity.. so distance / time difference

	    diff_distance_left = diff_tick_left * distancePerTick
	    diff_distance_right = diff_tick_right  * distancePerTick

	    time_diff = (current_time-last_time).to_sec()

	    #need to verify whether linear or angular - linear (verified)
	    feedback_vl = diff_distance_left/time_diff
	    feedback_vr = diff_distance_right/time_diff

	    #calculation of actual pose and linear angular velocities 
	    #from position encoded data 
	    #based on cpcr notes about base foot print frme
	    actual_vehicle_linear_vel = (feedback_vr+feedback_vl)/2    ## v - vehicle linear velocity
	    actual_vehicle_angular_vel = (feedback_vr-feedback_vl)/dist_bw_wheels  ## omega or theta_dot 
	    

	    #about odom frame
            dtheta = actual_vehicle_angular_vel * time_diff 
	    dx = actual_vehicle_linear_vel*time_diff * cos(actual_theta)
	    dy = actual_vehicle_linear_vel*time_diff * sin(actual_theta)
	    
	    actual_x += dx #actual position wrt to odom( home pose 0,0,0 )
	    actual_y += dy
	    actual_theta += dtheta
            
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, actual_theta)

            previous_left_tick_encoder = tick_left_encoder
            previous_right_tick_encoder = tick_right_encoder
	    #publish tf
 	
#sendTransform(self, translation, rotation, time, child, parent)
#Broadcast the transformation from tf frame child to parent on ROS topic "/tf".
            odom_trans = TransformStamped() 
  	    odom_trans.header.stamp = current_time
            odom_trans.header.frame_id = "odom"
            odom_trans.child_frame_id = "base_footprint"
            #wrt odom
            odom_trans.transform.translation.x = actual_x
            odom_trans.transform.translation.y = actual_y
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = odom_quat
            #send the transform
            odom_broadcaster.sendTransform((actual_x,actual_y,0),odom_quat,current_time,"base_footprint","odom");
            #Odometry message
	    odom = Odometry()
	    odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            #set the position
            odom.pose.pose.position.x = actual_x
            odom.pose.pose.position.y = actual_y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = Quaternion(*odom_quat)
            #set the velocity
            odom.child_frame_id = "base_footprint"
            #wrt base_footprint
            odom.twist.twist.linear.x = (actual_vehicle_linear_vel)
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = actual_vehicle_angular_vel
	    
#publishing odometry data
	    odometry_pub.publish(odom)
	    
            #setting joint states
            joint_state = JointState()
    	    joint_state.header = Header()
   	    joint_state.header.stamp = rospy.Time.now()
   	    joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
            joint_state.position = [(tick_left_encoder*2*pi/ticks_for_full_wheel_rotation),(tick_right_encoder*2*pi/ticks_for_full_wheel_rotation)]
            joint_state_pub.publish(joint_state)

    	    last_time = current_time
 
if __name__ == '__main__':
    listener()
