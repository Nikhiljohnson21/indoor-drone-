#!/usr/bin/env python3

import subprocess

def arm_motors():
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
                subprocess.run(["rosrun", "mavros", "mavsys", "mode", "-c", "2"])
                print("guided mode")
            except subprocess.CalledProcessError:
                print("Error while changing mode.")   

        elif user_input == 'loiter':
            try:
                subprocess.run(["rosrun", "mavros", "mavsys", "mode", "-c", "3"])
                print("loiter mode")
            except subprocess.CalledProcessError:
                print("Error while changing mode.")    
        elif user_input == 'stop':
            break        
        elif user_input == 'takeoff':
            try:
                altitude = 10.0  # Set the desired takeoff altitude in meters
                latitude = 47.12345  # set the actual latitude
                longitude = -122.67890  # set the actual longitude
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

if __name__ == '__main__':
    arm_motors()
