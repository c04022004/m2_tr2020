#!/usr/bin/env python
import math
from math import pi
import rospy
import numpy as np
import scipy.spatial as spatial
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from laser_line_extraction.msg import LineSegment, LineSegmentList


try_spot_side_dist = 0.475
try_spot_base_line_dist = 0.58
try_spot_hypot_line_dist = 0.75

corners_pub = None
hypot_line_pub = None
try_spot_center_pub = None
shift_z = pi/2
tolerance = 0.15
intercept_dist_tolerance = 0.7
diff_to_try_spot_dim_tolerance = 0.05


class Line:
    def __init__(self, x0, y0, x1, y1, shift_z=0):
        self.pt0 = (x0, y0)
        self.pt1 = (x1, y1)
        self.shift_z = shift_z
        self.angle_rad = math.atan2(self.yDiff(), self.xDiff()) + shift_z
        self.angle_rad += pi if self.angle_rad < 0.0 else 0.0 # Confine the angle range to [0, pi]
        self.angle_degree = self.angle_rad*180/pi
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


def extend_line_dist(closer_pt, further_pt, line_len, target_len):
    extended_x = closer_pt[0] + (further_pt[0]-closer_pt[0])/line_len*target_len
    extended_y = closer_pt[1] + (further_pt[1]-closer_pt[1])/line_len*target_len
    return (extended_x, extended_y)


def line_segments_cb(lines_msg):
    #         -----
    #         |   |
    #     -----   -----
    #
    #          RP
    # ^ x
    # |
    # ---> y
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
            # Adjacent lines of rectangle should be right angle 
            if abs(extracted_line.angle_degree - lines[i].angle_degree) < 80.0:
                continue
            if abs(extracted_line.angle_degree - lines[i].angle_degree) > 100.0:
                continue
            intercept = line_intersection(extracted_line, lines[i])
            if intercept[0] is None:
                continue
            # Ignore intercepts that are too far away
            if np.hypot(intercept[0],intercept[1]) > 1.5:
                continue
            lineA_0 = np.hypot(intercept[0]-extracted_line.pt0[0], intercept[1]-extracted_line.pt0[1])
            lineA_1 = np.hypot(intercept[0]-extracted_line.pt1[0], intercept[1]-extracted_line.pt1[1])
            if lineA_0 > intercept_dist_tolerance and lineA_1 > intercept_dist_tolerance:
                continue
            lineB_0 = np.hypot(intercept[0]-lines[i].pt0[0], intercept[1]-lines[i].pt0[1])
            lineB_1 = np.hypot(intercept[0]-lines[i].pt1[0], intercept[1]-lines[i].pt1[1])
            if lineB_0 > intercept_dist_tolerance and lineB_1 > intercept_dist_tolerance:
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

    markerArray = MarkerArray()
    for idx, point in enumerate(result_pts):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.id = idx
        marker.type = marker.SPHERE
        marker.action = marker.MODIFY
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
    if diff_to_try_spot_len < diff_to_try_spot_wid and diff_to_try_spot_len < diff_to_try_spot_dim_tolerance:
        is_base_line = True
        pt_polar_form = nearest_line.cartesian2PolarRad()
        pt_to_pub = Point(pt_polar_form[0], pt_polar_form[1], 0)
        # base_line_pub.publish(pt_to_pub)
    if diff_to_try_spot_wid < diff_to_try_spot_len and diff_to_try_spot_wid < diff_to_try_spot_dim_tolerance:
        is_side_line = True
        pt_polar_form = nearest_line.cartesian2PolarRad()
        pt_to_pub = Point(pt_polar_form[0], pt_polar_form[1], 0)
        # side_line_pub.publish(pt_to_pub)

    try_spot_hypot_list = []
    if is_base_line or is_side_line:
        for line in lines:
            # Remove outliers that are longer than expected length
            if is_base_line and line.length>(try_spot_side_dist+diff_to_try_spot_dim_tolerance):
                continue
            if is_side_line and line.length>(try_spot_base_line_dist+diff_to_try_spot_dim_tolerance):
                continue
            pt0_to_intercept0_dist = np.hypot(line.pt0[0]-p0[0], line.pt0[1]-p0[1])
            pt0_to_intercept1_dist = np.hypot(line.pt0[0]-p1[0], line.pt0[1]-p1[1])
            pt1_to_intercept0_dist = np.hypot(line.pt1[0]-p0[0], line.pt1[1]-p0[1])
            pt1_to_intercept1_dist = np.hypot(line.pt1[0]-p1[0], line.pt1[1]-p1[1])
            # Remove lines connected to the 2 intercepts
            if pt0_to_intercept0_dist<tolerance and pt1_to_intercept1_dist<tolerance:
                continue
            if pt0_to_intercept1_dist<tolerance and pt1_to_intercept0_dist<tolerance:
                continue
            furthest_pt = None
            opp_intercept_pt = None
            if pt0_to_intercept0_dist<tolerance or pt0_to_intercept1_dist<tolerance:
                opp_intercept_idx = 1 if pt0_to_intercept0_dist<tolerance else 0
                opp_intercept_pt = p0 if opp_intercept_idx==0 else p1
                pt0_to_origin_dist = np.hypot(line.pt0[0], line.pt0[1])
                opp_intercept_pt_to_origin_dist = np.hypot(opp_intercept_pt[0], opp_intercept_pt[1])
                # Remove presumed base_line if it is closer than side line
                if pt0_to_origin_dist<opp_intercept_pt_to_origin_dist and is_side_line:
                    continue
                if is_base_line:
                    furthest_pt = extend_line_dist(line.pt0, line.pt1, line.length, try_spot_side_dist)
                else:
                    furthest_pt = extend_line_dist(line.pt0, line.pt1, line.length, try_spot_base_line_dist)
            if pt1_to_intercept0_dist<tolerance or pt1_to_intercept1_dist<tolerance:
                opp_intercept_idx = 1 if pt1_to_intercept0_dist<tolerance else 0
                opp_intercept_pt = p0 if opp_intercept_idx==0 else p1
                pt1_to_origin_dist = np.hypot(line.pt1[0], line.pt1[1])
                opp_intercept_pt_to_origin_dist = np.hypot(opp_intercept_pt[0], opp_intercept_pt[1])
                # Remove presumed base_line if it is closer than side line
                if pt1_to_origin_dist<opp_intercept_pt_to_origin_dist and is_side_line:
                    continue
                if is_base_line:
                    furthest_pt = extend_line_dist(line.pt1, line.pt0, line.length, try_spot_side_dist)
                else:
                    furthest_pt = extend_line_dist(line.pt1, line.pt0, line.length, try_spot_base_line_dist)
            if furthest_pt is None:
                continue
            try_spot_hypot_line = Line(furthest_pt[0], furthest_pt[1], opp_intercept_pt[0], opp_intercept_pt[1], shift_z)
            if try_spot_hypot_line.length < (try_spot_hypot_line_dist-diff_to_try_spot_dim_tolerance) or \
            try_spot_hypot_line.length > (try_spot_hypot_line_dist+diff_to_try_spot_dim_tolerance):
                continue
            try_spot_hypot_list.append(try_spot_hypot_line)

    markerArray = MarkerArray()
    for idx, try_spot_hypot_line in enumerate(try_spot_hypot_list):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.id = idx
        marker.type = marker.LINE_STRIP
        marker.action = marker.MODIFY
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        point = Point()
        point.x = try_spot_hypot_line.pt0[0]
        point.y = try_spot_hypot_line.pt0[1]
        marker.points.append(point)
        point = Point()
        point.x = try_spot_hypot_line.pt1[0]
        point.y = try_spot_hypot_line.pt1[1]
        marker.points.append(point)
        marker.lifetime = rospy.rostime.Duration(0.2)
        markerArray.markers.append(marker)
    hypot_line_pub.publish(markerArray)

    if len(try_spot_hypot_list)==0:
        return
    try_spot_center_list = []
    for hypot_line in try_spot_hypot_list:
        try_spot_center_list.append(hypot_line.center())
    try_spot_center_np = np.asarray(try_spot_center_list)
    center_pt = pts_centeroid(try_spot_center_np)
    point = Point()
    point.x = center_pt[0]
    point.y = center_pt[1]
    try_spot_center_pub.publish(point)


rospy.init_node("lidar_try_spot_detector")
corners_pub = rospy.Publisher("/corners_marker", MarkerArray, queue_size=1)
hypot_line_pub = rospy.Publisher("/hypot_line_marker", MarkerArray, queue_size=1)
try_spot_center_pub = rospy.Publisher("/try_spot_center", Point, queue_size=1)
rospy.Subscriber("/line_segments", LineSegmentList, line_segments_cb)


rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
