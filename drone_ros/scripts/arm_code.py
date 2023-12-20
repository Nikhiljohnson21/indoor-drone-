#!/usr/bin/env python3
import rospy
import subprocess
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8

def arm_motors():
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)  # Create a publisher for velocity commands
     #pub = rospy.Publisher('directions', Int8, queue_size=10)
    rospy.init_node('qrcode_scan_node', anonymous=True)
    rate = rospy.Rate(10)

    while True: 
        user_input = input("Enter your command: ")
        if user_input == 'arm':
            try:
                subprocess.run(["rosrun", "mavros", "mavsafety", "arm"])
                print("Motors armed!")
            except subprocess.CalledProcessError:
                print("Error while arming motors.")

        elif user_input == 'stabilize':
            try:
                subprocess.run(["rosrun", "mavros", "mavsys", "mode", "-c", "1"])
                print("stabilize mode")
            except subprocess.CalledProcessError:
                print("Error while changing mode.")
        elif user_input == 'guided':
            try:
                subprocess.run(["rosrun", "mavros", "mavsys", "mode", "-c", "3"])
                print("guided mode")
            except subprocess.CalledProcessError:
                print("Error while changing mode.")   

        elif user_input == 'loiter':
            try:
                subprocess.run(["rosrun", "mavros", "mavsys", "mode", "-c", "6"])
                print("loiter mode")
            except subprocess.CalledProcessError:
                print("Error while changing mode.")    
        elif user_input == 'stop':
            break        
        elif user_input == 'takeoff':
            try:
                altitude = 10.0  # Set the desired takeoff altitude in meters
                latitude = 47.12345  # Replace with actual latitude
                longitude = -122.67890  # Replace with actual longitude
                subprocess.run(["rosrun", "mavros", "mavcmd", "takeoff", "0.0", "0.0", str(latitude), str(longitude), str(altitude)])
                print("Taking off to altitude:", altitude)
            except subprocess.CalledProcessError:
                print("Error while taking off.")
        elif user_input == 'disarm':
            try:
                subprocess.run(["rosrun", "mavros", "mavsafety", "disarm"])
                print("Motors disarmed!")
            except subprocess.CalledProcessError:
                print("Error while disarming motors.")     
        elif user_input == 'forward':
                twist = TwistStamped()
                twist.twist.linear.x = 1.0 #meter/second
                velocity_pub.publish(twist)
                print("Moving forward")
                
        elif user_input == 'backward':
                twist = TwistStamped()
                twist.twist.linear.x = -1.0 #meter/second
                velocity_pub.publish(twist)
                print("Moving backward")
                
        elif user_input == 'left':
                twist = TwistStamped()
                twist.twist.linear.y = 1.0 #meter/second
                velocity_pub.publish(twist)
                print("Moving left")
                
        elif user_input == 'right':
                twist = TwistStamped()
                twist.twist.linear.y = -1.0 #meter/second
                velocity_pub.publish(twist)
                print("Moving right")          

if __name__ == '__main__':
    arm_motors()
