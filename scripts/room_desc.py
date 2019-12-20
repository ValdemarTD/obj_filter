#!/usr/bin/env python
from shapely.geometry import *

#Class definition for a room object. Intent is to use Shapely to determine whether
#a given point is within a certain distance of the walls of a room or outside of
#the room. Purpose is to filter out scan data hitting the walls to make object
#detection easier

class Room:
    def __init__(self):
        self.points = []
        self.edge_boxes = []
        self.edge_margin = 0.45

    #Takes in an array of XY coordinates (or potentially XYZ, which are interpreted
    #in the same way by Shapely) and converts it to a series of XY Point objects
    def fill_points_array(self, point_arr):
        self.points = []
        for new_point in point_arr:
            self.points.append([new_point[0], new_point[1]])

    #Updates the room polygon with the most recent set of points
    def update_poly(self):
        self.room_poly = Polygon(self.points)
        self.make_checked_box()

    #Manually sets the room polygon from another polygon
    def set_poly(self, poly_in):
        self.room_poly = poly_in
        self.make_checked_box()

    #Creates a polygon inside the room object. We offset inwards by the number
    #of meters specified by self.edge_margin to account for sensor noise
    def make_checked_box(self):
        self.checked_box = Polygon(self.room_poly.exterior.parallel_offset(self.edge_margin, 'right'))


    #Checks if a point is past the wall or in the wall bounding box
    def has_inside(self, point_to_check):
        new_point = Point(point_to_check[0], point_to_check[1])
        return self.checked_box.contains(new_point)
