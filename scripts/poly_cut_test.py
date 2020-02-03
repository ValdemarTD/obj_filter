#!/usr/bin/env python
from room_desc import Room
import math
from random import *
from time import *


#This amounts to a spike.

class LaserStruct:
    def __init__(self):
        self.max_range = 10.0
        self.min_angle = -(math.pi/2)
        self.max_angle = math.pi/2
        self.range_count = 200.0

class PoseStruct:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.line_array = []

start_time = time()

def gen_points(l_struct, pose_struct):
    point_array = [(pose_struct.x, pose_struct.y)]
    point_count = int(l_struct.range_count / 10.0)
    angle_increment = float(l_struct.max_angle - l_struct.min_angle) / float(point_count)
    for i in range(point_count + 1):
        theta = l_struct.min_angle + i * angle_increment + pose_struct.yaw
        x = l_struct.max_range * math.cos(theta) + pose_struct.x
        y = l_struct.max_range * math.sin(theta) + pose_struct.y
        point_array.append((x,y))
    return point_array

def mcl_pose_generator(room_desc, point_count):
    #Seeds our RNG
    seed()
    bounding_points = room_desc.get_bounding_box_points()
    xs = []
    ys = []
    for point in bounding_points:
        xs.append(point[0])
        ys.append(point[1])
    min_x = min(xs)
    min_y = min(ys)
    max_x = max(xs)
    max_y = max(ys)
    x_diff = max_x - min_x
    y_diff = max_y - min_y
    poses = []
    while len(poses) < point_count:
        x = random() * x_diff + min_x
        y = random() * y_diff + min_y
        if room_desc.has_inside((x, y), inner_poly=False):
            pose = PoseStruct()
            pose.x = x
            pose.y = y
            pose.yaw = 2 * math.pi * random()
            poses.append(pose)
    return poses

laser_struct = LaserStruct()

points = [(0,0), (0,6), (8,6), (8,0)]

room = Room()
room.fill_points_array(points)
room.update_poly()
poses_to_check = mcl_pose_generator(room, 150)


i = 1
for pose in poses_to_check:
    print str(i) + ": (" + str(pose.x) + ", " + str(pose.y) + ", " + str(pose.yaw) + ")"
    print str(room.get_room_segment(boundary_points=gen_points(laser_struct, pose))) + "\n"
    i += 1

end_time = time()
print "\nTotal time taken: " + str(end_time - start_time)
