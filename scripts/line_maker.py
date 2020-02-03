#!/usr/bin/env python
import rospy as rp
import tf
import math
from room_desc import Room
from object_filter import ObjectFilter
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


class LineMaker:
    def __init__(self):
        self.room = None
        self.listener = tf.TransformListener()

    #Function to set the room to be used for line handling
    def set_room(self, new_room):
        self.room = new_room

    #Function to change the frame of a LaserScan to (usually) the base_footprint frame,
    #likely spoofed for Monte Carlo Localization purposes
    def change_frame(self, pose, scan):
        frame_1 = pose.header.frame
        frame_2 = scan.header.frame

        #Finds the transform from scanner to base frame, then takes into account
        #the base frame's position.
        (translation, rotation) = self.listener.lookupTransform(frame_2, frame_1, rp.Time(0))
        translation.x += pose.pose.x
        translation.y += pose.pose.y
        translation.z += pose.pose.z

        #Must add Euler angles, not Quaternions 
