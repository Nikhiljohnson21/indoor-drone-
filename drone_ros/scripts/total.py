#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


MAX_DISTANCE = 0.45


obstacle_in_front = False

def scan_callback(data):
    global obstacle_in_front
    
    
    min_distance_index = data.ranges.index(min(data.ranges))
    
    
    min_distance = min(data.ranges)
    
    
    angle_min = data.angle_min  
    angle_increment = data.angle_increment  
    angle_of_min_distance = angle_min + min_distance_index * angle_increment
    
    
    if min_distance < MAX_DISTANCE:
        if angle_of_min_distance < 0:
            if abs(angle_of_min_distance) < 1.57:  
                obstacle_in_front = True
            else:
                obstacle_in_front = False
        else:
            obstacle_in_front = False
    
    
    if obstacle_in_front:
        user_input = 'left'
    else:
        user_input = 'forward'
    
    
    publish_velocity(user_input)

def publish_velocity(user_input):
    velocity_pub = rospy.Publisher('/your_drone/velocity_command_topic', TwistStamped, queue_size=10)
    twist = TwistStamped()
    
    if user_input == 'forward':
        twist.twist.linear.x = 1.0  
    elif user_input == 'left':
        twist.twist.linear.y = 1.0  
    
    velocity_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('obstacle_detection_node')
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()
