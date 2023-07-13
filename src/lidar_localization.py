#!usr/bin/env python3

#This Code is an attempt to create lidar_localization code for the purpose of localization of the robot using lidar data

#Down Below is where we import the libraries that we will use

import rospy
#By importing rospy, we access all the entire ROS Library

import numpy as np
#Numpy is a library that allows us to do math with arrays

import matplotlib.pyplot as plt
#Matplotlib is a library that allows us to plot data


from sensor_msgs.msg import LaserScan
#Sensor_msgs is a package that contains messages for commonly used sensors
#Our Lidar Only Sees in 2D, so we will only use the LaserScan Message

from std_msgs.msg import String
#std_msgs is a package that contains messages for common data types

import csv
#csimport csv
#csv is a library that allows us to read and write csv files



#The First Thing We Want To Do is Extract the Data from the Lidar Sensor
#In Essense Will be similar to the #This Code is an attempt to create lidar_localization node for the purpose of localization of the robot using lidar data
code in scan_practice.py

def lidarscan_to_data(Scan_Message: LaserScan) -> np.ndarray:
#The Data will be in the form of a numpy array
#We will have to utilze polar coordinates to convert the data to cartesian coordinates

    angles=np.linspace(Scan_Message.angle_min, Scan_Message.angle_max, len(Scan_Message.ranges))
    #Now that we have the angles, we can convert the data to cartesian coordinates

    x=Scan_Message.ranges*np.cos(angles)
    y=Scan_Message.ranges*np.sin(angles)

    # Put in matrix form
    return np.vstack((x, y)).T




#nplinspace is a function that creates a numpy array of evenly spaced numbers







#Use Class When storing data

#During the Prtotype Save The Data. in the Final Code Dont. 


#In Final Code, Put Data in variables
