#!/usr/bin/env python3

import cv2
import rospy
from std_msgs.msg import Int8
from cv_bridge import CvBridge
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool  # Import the service
import subprocess
import sys
sys.path.append('/usr/local/lib/python3.8/dist-packages')


def decode_qr_code(image):
    quirc = cv2.QRCodeDetector()
    decoded_info, points, straight_qrcode = quirc.detectAndDecode(image)
    return decoded_info

def mavlink_commands():
    pub = rospy.Publisher('/mavros/cmd/arming', PositionTarget, queue_size=10)
    rospy.init_node('mavlink_commands_node', anonymous=True)
    rate = rospy.Rate(10)
    
    # Create a service proxy for arming and disarming
    #arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    
    cap = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()
    bridge = CvBridge()

    forward_command_received = False

    while not rospy.is_shutdown():
        _, img = cap.read()
        command = ''
        if img is not None:
           command, _, _ = detector.detectAndDecode(img)
             
        
            
        if command == 'move_forward':
           msg = PositionTarget()
           msg.header.stamp = rospy.Time.now()
           msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
           msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
           msg.velocity.x = 1.0  # Desired forward velocity in m/s
           

           pub.publish(msg)
           rospy.loginfo("Moving the vehicle forward")
           forward_command_received = True  # Set the flag to indicate forward command received
            
        elif command == '0':
             try:
                subprocess.run(["rosrun", "mavros", "mavsafety", "arm"])
                print("Motors armed!")
             except subprocess.CalledProcessError:
                print("Error while arming motors.")

        
        elif command == '1':
             try:
                subprocess.run(["rosrun", "mavros", "mavsafety", "disarm"])
                print("Motors disarmed!")
             except subprocess.CalledProcessError:
                print("Error while disarming motors.")   

        elif command == '3':
            try:
                subprocess.run(["rosrun", "mavros", "mavsys", "mode", "-c", "1"])
                print("stabilize mode")
            except subprocess.CalledProcessError:
                print("Error while changing mode.")  

        elif command == '6':
            try:
                altitude = 10.0  # Set the desired takeoff altitude in meters
                latitude = 47.12345  # set the actual latitude
                longitude = -122.67890  # set the actual longitude
                subprocess.run(["rosrun", "mavros", "mavcmd", "takeoff", "0.0", "0.0", str(latitude), str(longitude), str(altitude)])
                print("Taking off to altitude:", altitude)
            except subprocess.CalledProcessError:
                print("Error while taking off.")             


        elif command == 'move_left':
            msg = PositionTarget()
            msg.header.stamp = rospy.Time.now()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
            msg.velocity.y = -1.0  # Desired leftward velocity in m/s

            pub.publish(msg)
            rospy.loginfo("Moving the vehicle left")
            forward_command_received = False  # Reset the flag
                
        elif command == 'move_right':
            msg = PositionTarget()
            msg.header.stamp = rospy.Time.now()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                                PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
            msg.velocity.y = 1.0  # Desired rightward velocity in m/s

            pub.publish(msg)
            rospy.loginfo("Moving the vehicle right")
            forward_command_received = False  # Reset the flag

            # Stop moving if a new QR code is scanned
        if forward_command_received and command != 'move_forward':
            msg = PositionTarget()
            msg.velocity.x = 0.0  # Stop the drone's motion
            pub.publish(msg)
            rospy.loginfo("Stopping the vehicle")
            forward_command_received = False  # Reset the flag
        
        rate.sleep()

    #cap.release()

if __name__ == '__main__':
    try:
        mavlink_commands()
    except rospy.ROSInterruptException:
        pass

        
        
        
