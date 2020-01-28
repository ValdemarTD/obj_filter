#!/usr/bin/env python
from room_desc import Room


points = [(0,0), (0,6), (8,6), (8,0)]
cut_points = [(1,1), (1,12), (2,13), (3,3), (4, 12), (5, 0.5), (9, 0.4), (5, 0.25)]

room = Room()
room.fill_points_array(points)
room.update_poly()
room.get_room_segment(boundary_points=cut_points)
