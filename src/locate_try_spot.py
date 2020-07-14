#!/usr/bin/env python
import math
from math import pi
import rospy
import numpy as np
import scipy.spatial as spatial
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from laser_line_extraction.msg import LineSegment, LineSegmentList


corners_pub = None
base_line_pub = None
side_line_pub = None
try_spot_center_pub = None
shift_z = pi/2
tolerance = 0.05


class Line:
    def __init__(self, x0, y0, x1, y1, shift_z=0):
        self.pt0 = (x0, y0)
        self.pt1 = (x1, y1)
        self.shift_z = shift_z
        # TODO: Confine angle_rad to a particlar range
        self.angle_rad = math.atan2(self.yDiff(), self.xDiff()) + shift_z
        self.angle_degree_abs = abs(self.angle_rad*180/pi)
        self.length = np.hypot(self.pt1[0]-self.pt0[0],self.pt1[1]-self.pt0[1])

    def xDiff(self):
        return self.pt0[0] - self.pt1[0]

    def yDiff(self):
        return self.pt0[1] - self.pt1[1]

    def slope(self):
        if self.pt0[0] == self.pt1[0]:
            return np.inf
        if self.pt0[1] == self.pt1[1]:
            return 0.0
        return (self.pt1[1]-self.pt0[1]) / (self.pt1[0]-self.pt0[0])

    def center(self):
        return ( (self.pt0[0]+self.pt1[0])/2, (self.pt0[1]+self.pt1[1])/2 )

    def cartesian2PolarRad(self):
        return (self.length, self.angle_rad)


# Copied and/or modified from https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
def line_intersection(line0, line1):
    xdiff = ( line0.xDiff(), line1.xDiff() )
    ydiff = ( line0.yDiff(), line1.yDiff() )

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return (None, None)

    d = ( det(line0.pt0,line0.pt1), det(line1.pt0,line1.pt1) )
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return (x, y)


# Copied and/or modified from https://stackoverflow.com/questions/23020659/fastest-way-to-calculate-the-centroid-of-a-set-of-coordinate-tuples-in-python-wi
def pts_centeroid(pts_np):
    length = pts_np.shape[0]
    sum_x = np.sum(pts_np[:, 0])
    sum_y = np.sum(pts_np[:, 1])
    return (sum_x/length, sum_y/length)


def line_segments_cb(lines_msg):
    lines = []
    lines_count = 0
    interceptions = []
    result_pts = []
    
    for line in lines_msg.line_segments:
        extracted_line = Line(line.start[0], line.start[1],
            line.end[0], line.end[1], shift_z)
        lines.append(extracted_line)
        lines_count += 1
        if lines_count < 2:
            continue
        for i in range(lines_count-1):
            if abs(extracted_line.angle_degree_abs - lines[i].angle_degree_abs) < 45.0:
                continue
            intercept = line_intersection(extracted_line, lines[i])
            if intercept[0] is None:
                continue
            if intercept[0]>2.0 or intercept[1]>2.0:
                continue
            lineA_0 = np.hypot(intercept[0]-extracted_line.pt0[0], intercept[1]-extracted_line.pt0[1])
            lineA_1 = np.hypot(intercept[0]-extracted_line.pt1[0], intercept[1]-extracted_line.pt1[1])
            if lineA_0 > 0.7 and lineA_1 > 0.7:
                continue
            lineB_0 = np.hypot(intercept[0]-lines[i].pt0[0], intercept[1]-lines[i].pt0[1])
            lineB_1 = np.hypot(intercept[0]-lines[i].pt1[0], intercept[1]-lines[i].pt1[1])
            if lineB_0 > 0.7 and lineB_1 > 0.7:
                continue
            interceptions.append(intercept)

    if len(interceptions)==0:
        return
    intercepts_np = np.asarray(interceptions)
    point_tree = spatial.cKDTree(intercepts_np)
    clustered_pts = []
    for i in range( len(intercepts_np) ):
        if i in clustered_pts:
            continue
        group_idxs = point_tree.query_ball_point(intercepts_np[i], tolerance)
        result_pts.append(pts_centeroid(point_tree.data[group_idxs]) )
        clustered_pts += group_idxs

    # Remove outliers
    tmp_pts = []
    for point in result_pts:
        distance_to_origin = np.hypot(point[0],point[1])
        # Only for locating try_spot
        if distance_to_origin > 1.5:
            continue
        tmp_pts.append(point)
    result_pts = tmp_pts

    markerArray = MarkerArray()
    for idx, point in enumerate(result_pts):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.id = idx
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.lifetime = rospy.rostime.Duration(0.2)
        markerArray.markers.append(marker)
    corners_pub.publish(markerArray)

    if len(result_pts) < 2:
        return
    pts_distance = [np.hypot(point[0],point[1]) for point in result_pts]
    pts_distance_np = np.asarray(pts_distance)
    sorted_idx = np.argsort(pts_distance_np)
    p0 = result_pts[sorted_idx[0]]
    p1 = result_pts[sorted_idx[1]]
    nearest_line = Line(p0[0], p0[1], p1[0], p1[1], shift_z)
    diff_to_try_spot_len = abs(nearest_line.length - 0.58)
    diff_to_try_spot_wid = abs(nearest_line.length - 0.475)
    is_base_line = False
    is_side_line = False
    if diff_to_try_spot_len < diff_to_try_spot_wid and diff_to_try_spot_len < tolerance:
        is_base_line = True
        pt_polar_form = nearest_line.cartesian2PolarRad()
        pt_to_pub = Point(pt_polar_form[0], pt_polar_form[1], 0)
        base_line_pub.publish(pt_to_pub)
        print(pt_polar_form)
    if diff_to_try_spot_wid < diff_to_try_spot_len and diff_to_try_spot_wid < tolerance:
        is_side_line = True
        pt_polar_form = nearest_line.cartesian2PolarRad()
        pt_to_pub = Point(pt_polar_form[0], pt_polar_form[1], 0)
        side_line_pub.publish(pt_to_pub)
        print(pt_polar_form)
    if len(result_pts) < 3:
        return
    if is_base_line or is_side_line:
        for idx in sorted_idx[2:]:
            line = Line(result_pts[idx][0], result_pts[idx][1], p0[0], p0[1], shift_z)
            diff_to_try_spot_side = abs(line.length - 0.475) if is_base_line else \
                abs(line.length - 0.58)
            diff_to_try_spot_hypot = abs(line.length - 0.7497) # sqrt(0.58**2 + 0.475**2)
            try_spot_center = None
            if diff_to_try_spot_hypot < tolerance:
                try_spot_center = line.center()
            if diff_to_try_spot_side < tolerance:
                line = Line(result_pts[idx][0], result_pts[idx][1], 
                    p1[0], p1[1], shift_z)
                try_spot_center = line.center()
            if try_spot_center is None:
                continue
            length_to_center = np.hypot(try_spot_center[0],try_spot_center[1])
            # TODO: Confine angle_rad to a particlar range
            angle_rad = math.atan2(-try_spot_center[1], -try_spot_center[0]) + shift_z
            # print((length_to_center, angle_rad))
            pt_to_pub = Point(length_to_center, angle_rad, 0)
            try_spot_center_pub.publish(pt_to_pub)
            break


rospy.init_node("lidar_try_spot_detector")
corners_pub = rospy.Publisher("/corners_marker", MarkerArray, queue_size=1)
base_line_pub = rospy.Publisher("/try_spot_base_line", Point, queue_size=1)
side_line_pub = rospy.Publisher("/try_spot_side_line", Point, queue_size=1)
try_spot_center_pub = rospy.Publisher("/try_spot_center", Point, queue_size=1)
rospy.Subscriber("/line_segments", LineSegmentList, line_segments_cb)


rate = rospy.Rate(100)
while not rospy.is_shutdown():
    rate.sleep()
