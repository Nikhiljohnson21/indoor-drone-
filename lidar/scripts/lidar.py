#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(data):
    # Find the index of the minimum distance in the scan data
    min_distance_index = data.ranges.index(min(data.ranges))
    
    # Calculate the angle associated with the minimum distance
    angle_min = data.angle_min  # Starting angle of the scan
    angle_increment = data.angle_increment  # Angular resolution
    angle_of_min_distance = angle_min + min_distance_index * angle_increment

    # Define a threshold distance for obstacle detection (adjust as needed)
    threshold_distance = 0.5  # You can adjust this distance

    # Check if the minimum distance is less than the threshold
    if data.ranges[min_distance_index] < threshold_distance:
        # An obstacle is detected, print a message
        rospy.loginfo("Obstacle detected!")

if __name__ == '__main__':
    rospy.init_node('obstacle_detection_node')
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()
