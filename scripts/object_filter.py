#!/usr/bin/env python
import rospy as rp
from room_desc.py import Room
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class Object_Filter:
    def __init__(self):
        #Set up publisher and subscriber
        rp.Subscriber("localized_pointcloud", PointCloud, self.pc_callback)
        self.filtered_pub = rp.Publisher("filtered_poincloud", PointCloud, queue_size = 1)

        #Set up room object. Hard coded values for now.
        self.room = Room()
        self.room.fill_points_array([(0,0), (0,6), (8,6), (8,0)])
        self.room.update_poly()

    #Callback function for localized_pointcloud topic. Takes in an unfiltered but localized
    #pointcloud, copies over the
    def pc_callback(self, data):
        new_points = []
        cloud_out = PointCloud()
        cloud_out.header = data.header
        cloud_out.channels = data.channels

        #Goes through all points and only includes ones that aren't close to or
        #outside of the walls indicated in the room object
        for point in data.points:
            if self.room.has_inside((point.x, point.y)):
                new_points.append(point)
        cloud_out.points = new_points

        #Publishes updated cloud
        self.filtered_pub.publish(cloud_out)


if __name__ == "__main__":
    rp.init_node('object_filter', anonymous=True)
    object_filter = Object_Filter()
    while not rp.is_shutdown():
        rp.spin()
