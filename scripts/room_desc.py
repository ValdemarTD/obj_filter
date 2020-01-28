#!/usr/bin/env python
from shapely.geometry import *
from shapely.ops import split
from shapely import speedups
from numpy import array


#Enables Shapely Geometry speedups if they're available and not already enabled
#Should be enabled by default (as of Shapely 1.6.0), but hey, you never know.
if speedups.available and not speedups.enabled:
    speedups.enable()

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

    def get_room_segment(self, cutting_poly=None, boundary_points=None):
        #Makes sure that we have a polygon to cut with, either from being
        #provided with one or from being provided with points to make one.
        if cutting_poly == None:
            if boundary_points == None:
                print "Error: No cut polygon or boundary points specified."
            else:
                points = []
                for point in boundary_points:
                    points.append([point[0], point[1]])
                direction_check = LinearRing(points)
                cutting_poly = Polygon(points)

        #Cut function doesn't work on LinearRing so we have to convert it to LineString
        room_lines = LineString(self.room_poly.exterior)

        #Cut the room into pieces.
        cut_result = split(room_lines, cutting_poly)

        #Buffering by a tiny amount allows us to get consistent DE-9IM patterns
        #when comparing out polygon and lines
        test_poly = cutting_poly.buffer(0.001)

        #Set up an array to put all of our segments in.
        to_return = []
        for cut in cut_result:
            #Pattern to match was found by manually testing cuts to find a
            #consistent pattern in DE-9IM format using object.relate(object2)
            if test_poly.relate_pattern(cut, '102FF1FF2'):
                to_return.append(array(cut))
        return to_return
