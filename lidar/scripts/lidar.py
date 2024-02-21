#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(data):
    min_distance_index = data.ranges.index(min(data.ranges))
    
    angle_min = data.angle_min  
    angle_increment = data.angle_increment  
    angle_of_min_distance = angle_min + min_distance_index * angle_increment

    threshold_distance = 0.5  

    if data.ranges[min_distance_index] < threshold_distance:
        rospy.loginfo("Obstacle detected!")

if __name__ == '__main__':
    rospy.init_node('obstacle_detection_node')
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()
