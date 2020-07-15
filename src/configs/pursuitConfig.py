#!/usr/bin/env python
from math import pi
from geometry_msgs.msg import *
from m2_move_base.msg import *


POINT_S = (0.5, 9.5)
POINT_C = (1.1, 1.1)
POINT_1 = (5.6, 3.09+4.0*0.03+4*1.3)
POINT_2 = (5.6, 3.09+3.0*0.03+3*1.3)
POINT_3 = (5.6, 3.09+2.0*0.03+2*1.3)
POINT_4 = (5.6, 3.09+1.0*0.03+1*1.3)
POINT_5 = (5.6, 3.09+0.5*0.03+0*1.3)

CHK_PTS = [
    [ POINT_S, POINT_C ], # scene 1
    [ POINT_C,(3.50453,4.11369),(4.55161,5.41157),(5.19676,7.57945),POINT_1 ], # scene 2
    [ POINT_C,(2.65163,2.93525),(4.34558,4.87455),POINT_2 ], # scene 3
    [ POINT_C,(3.1, 3.57),POINT_3 ], # scene 4
    [ POINT_C,(2.45023,2.59568),(4.19525,3.97616),POINT_4 ], # scene 5
    [ POINT_C,(2.99331,3.31949),(4.56409,3.48977),POINT_5 ], # scene 6
    [ POINT_C, POINT_S ], # scene 7
]

class PurePursuitConfig: # Default setting of MATCH_RED
    speed_f = 2.5
    speed_b = 2.5
    radius_f = 1.0
    radius_b = 1.0

    spline_label = "cubic"
    spline_type = PurePursuitData.CUBIC
    raw_pts = []
    point_density = 1000

    velocity_z_kP = 3.0
    velocity_z_max = 2.0
    target_z = -pi/2

    lookahead_distance = 0.0
    vector_policy = PurePursuitData.CURVATURE_DEPENDENT_TWIST_WITH_TANGENTIAL_PID
    velocity_shift_kP = 6.0
    velocity_shift_kD = 0.0 # unstable!
    curvature_penalty_kP = 1.0

    stop_type = PurePursuitData.STOP_PID
    stop_pid_radius = 2.0
    stop_kP = 2.0
    stop_kD = 0.4

    def __init__(self, spline_label, raw_pts):
        self.spline_label = spline_label
        if spline_label == "cubic":
            self.spline_type = PurePursuitData.CUBIC
        elif spline_label == "bezier":
            self.spline_type = PurePursuitData.BEZIER
        self.raw_pts = raw_pts
        self.chk_pts = [Point(x=point[0], y=point[1]) for point in self.raw_pts]

    def knotsFlipX(self):
        self.chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in self.raw_pts]
        self.target_z = -self.target_z

    def goalConstructor(self, is_forward):
        pure_pursuit_data = PurePursuitData()
        pure_pursuit_data.label = self.spline_label
        pure_pursuit_data.spline_type = self.spline_type

        if is_forward:
            pure_pursuit_data.knots = self.chk_pts
        else:
            copied_pts = self.chk_pts[:]
            copied_pts.reverse()
            pure_pursuit_data.knots = copied_pts
        pure_pursuit_data.point_density = self.point_density
        
        pure_pursuit_data.velocity_xy_magnitude = self.speed_f if is_forward else self.speed_b

        pure_pursuit_data.velocity_z_kP = self.velocity_z_kP
        pure_pursuit_data.velocity_z_max = self.velocity_z_max
        pure_pursuit_data.target_z = self.target_z

        pure_pursuit_data.lookahead_distance = self.lookahead_distance
        pure_pursuit_data.vector_policy = self.spline_type
        pure_pursuit_data.velocity_shift_kP = self.velocity_shift_kP
        pure_pursuit_data.velocity_shift_kD = self.velocity_shift_kD
        pure_pursuit_data.curvature_penalty_kP = self.curvature_penalty_kP

        pure_pursuit_data.stop_type = self.stop_type
        pure_pursuit_data.stop_pid_radius = self.radius_f if is_forward else self.radius_b
        pure_pursuit_data.stop_kP = self.stop_kP
        pure_pursuit_data.stop_kD = self.stop_kD

        goal = SwitchModeGoal(target_mode=SwitchModeGoal().PURE_PURSUIT, pure_pursuit_data=pure_pursuit_data)
        return goal


scene1_cfg = PurePursuitConfig("cubic", CHK_PTS[0])
scene2_cfg = PurePursuitConfig("cubic", CHK_PTS[1])
scene3_cfg = PurePursuitConfig("cubic", CHK_PTS[2])
scene4_cfg = PurePursuitConfig("cubic", CHK_PTS[3])
scene5_cfg = PurePursuitConfig("cubic", CHK_PTS[4])
scene6_cfg = PurePursuitConfig("cubic", CHK_PTS[5])
scene7_cfg = PurePursuitConfig("cubic", CHK_PTS[6])
