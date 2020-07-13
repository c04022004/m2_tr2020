#!/usr/bin/env python
import math
import rospy
import numpy as np
import scipy.spatial as spatial
from visualization_msgs.msg import Marker, MarkerArray
from laser_line_extraction.msg import LineSegment, LineSegmentList


corners_pub = None


class Line:
    def __init__(self, x0, y0, x1, y1):
        self.pt0 = (x0, y0)
        self.pt1 = (x1, y1)
        self.angle_degree = abs(math.atan2(self.yDiff(), self.xDiff())*180/math.pi)
    
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
        extracted_line = Line(line.start[0],line.start[1],
            line.end[0],line.end[1])
        lines.append(extracted_line)
        lines_count += 1
        if lines_count < 2:
            continue
        for i in range(lines_count-1):
            if abs(extracted_line.angle_degree - lines[i].angle_degree) < 45.0:
                continue
            intercept = line_intersection(extracted_line, lines[i])
            if intercept[0] is None:
                continue
            if intercept[0]>2.0 or intercept[1]>2.0:
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
        group_idxs = point_tree.query_ball_point(intercepts_np[i], 0.1)
        result_pts.append(pts_centeroid(point_tree.data[group_idxs]) )
        clustered_pts += group_idxs

    markerArray = MarkerArray()
    for idx, point in enumerate(result_pts):
        if np.hypot(point[0],point[1]) > 1.5: # Only for locating try_spot
            continue
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


rospy.init_node("lidar_try_spot_detector")
corners_pub = rospy.Publisher("/try_spot_corners", MarkerArray, queue_size=1)
rospy.Subscriber("/line_segments", LineSegmentList, line_segments_cb)


rate = rospy.Rate(100)
while not rospy.is_shutdown():
    rate.sleep()
