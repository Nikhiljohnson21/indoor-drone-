#!/usr/bin/env python3

import subprocess
import time
import cv2
import rospy
from std_msgs.msg import Int8
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped

def arm_motors():

    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)  # Create a publisher for velocity commands

    #pub = rospy.Publisher('directions', Int8, queue_size=10)
    rospy.init_node('qrcode_scan_node', anonymous=True)
    rate = rospy.Rate(10)

    cap = cv2.VideoCapture("/dev/video0")
    detector = cv2.QRCodeDetector()
    bridge = CvBridge()
    
    while not rospy.is_shutdown():
        _, img = cap.read() 
        
        if img is not None:
            command, points, _ = detector.detectAndDecode(img)
            
            #if points is not None:
             #   points = points[0]
              #  for i in range(len(points)):
               #     pt1 = [int(val) for val in points[i]]
                #    pt2 = [int(val) for val in points[(i + 1) % 4]]
                 #   cv2.line(img, pt1, pt2, color=(0, 255, 0), thickness=6)
                  #  cv2.putText(img, command, (pt1[0], pt1[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)        
            cv2.imshow('Camera Feed', img)
        
            # arm motors -------------------------------------------------------------------------
            if command == 'arm':
                try:
                    subprocess.run(["rosrun", 
                                    "mavros", 
                                    "mavsafety", 
                                    "arm"])
                    print("Motors armed!")
                except subprocess.CalledProcessError:
                    print("Error while arming motors.")
                
            # stabilize mode ---------------------------------------------------------------------
            elif command == 'stabilize':
                try:
                    subprocess.run(["rosrun", 
                                    "mavros", 
                                    "mavsys", 
                                    "mode", 
                                    "-c", 
                                    "1"])
                    print("stabilize mode")
                except subprocess.CalledProcessError:
                    print("Error while changing mode.")
                
            # guided mode ------------------------------------------------------------------------        
            elif command == 'guided':
                try:
                    subprocess.run(["rosrun", 
                                    "mavros", 
                                    "mavsys", 
                                    "mode", 
                                    "-c", 
                                    "3"])
                    print("guided mode")
                except subprocess.CalledProcessError:
                    print("Error while changing mode.")   

            # loiter mode ------------------------------------------------------------------------
            elif command == 'loiter':
                try:
                    subprocess.run(["rosrun", 
                                    "mavros", 
                                    "mavsys", 
                                    "mode", 
                                    "-c", 
                                    "6"])
                    print("loiter mode")
                except subprocess.CalledProcessError:
                    print("Error while changing mode.")    
      
            # takeoff the drone ------------------------------------------------------------------
            elif command == 'takeoff':
                try:
                    subprocess.run(["rosrun", 
                                    "mavros", 
                                    "mavsafety", 
                                    "arm"])
                    print("Motors armed!")

                    subprocess.run(["rosrun", 
                                    "mavros", 
                                    "mavsys", 
                                    "mode", 
                                    "-c", 
                                    "6"])
                    print("loiter mode")
                    
                    altitude = 10.0                          # Set the desired takeoff altitude in meters
                    latitude = 47.12345                      # Replace with actual latitude
                    longitude = -122.67890                   # Replace with actual longitude
                    subprocess.run(["rosrun", 
                                    "mavros", 
                                    "mavcmd", 
                                    "takeoff", 
                                    "0.0", 
                                    "0.0", 
                                    str(latitude), 
                                    str(longitude), 
                                    str(altitude)])
                    print("Taking off to altitude:", altitude)
                except subprocess.CalledProcessError:
                    print("Error while taking off.")
                    
            # velocity control -------------------------------------------------------------------
            elif command == 'forward':
                twist = TwistStamped()
                twist.twist.linear.x = 1.0 #meter/second
                velocity_pub.publish(twist)
                print("Moving forward")
                
            elif command == 'backward':
                twist = TwistStamped()
                twist.twist.linear.x = -1.0 #meter/second
                velocity_pub.publish(twist)
                print("Moving backward")
                
            elif command == 'left':
                twist = TwistStamped()
                twist.twist.linear.y = 1.0 #meter/second
                velocity_pub.publish(twist)
                print("Moving left")
                
            elif command == 'right':
                twist = TwistStamped()
                twist.twist.linear.y = -1.0 #meter/second
                velocity_pub.publish(twist)
                print("Moving right")
             
            # disarm motors ---------------------------------------------------------------------       
            elif command == 'disarm':
                try:
                   subprocess.run(["rosrun", 
                                   "mavros", 
                                   "mavsafety", 
                                   "disarm"])
                   print("Motors disarmed!")
                except subprocess.CalledProcessError:
                   print("Error while disarming motors.") 
                
            # stop all the process and exit ------------------------------------------------------   
            elif command == 'stop':
                print("stoping process...")
                time.sleep(1)
                print("exited process")
                break
            
            command = " "
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break    
        rate.sleep()
        
    cap.release()     

if __name__ == '__main__':
    arm_motors()



