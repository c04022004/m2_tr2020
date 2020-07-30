#!/usr/bin/env python
from math import pi
from geometry_msgs.msg import *
from m2_move_base.msg import *

MATCH_NONE = 0
MATCH_RED  = 1
MATCH_BLUE = 2

POINT_S = (0.5, 9.5)
POINT_C = (1.1, 1.1)
POINT_D = (1.1, 3.5)
# end-pt x 6150 - 300(half robot) - 40(protector) 
# end-pt y 3090 + 30(white line)  + 1300(offset)
tryx_offset = 0.10 # reserve for overshoot
POINT_1 = (5.81-tryx_offset, 3.09+0.5*0.03+0*1.3)
POINT_2 = (5.81-tryx_offset, 3.09+1.0*0.03+1*1.3)
POINT_3 = (5.81-tryx_offset, 3.09+2.0*0.03+2*1.3)
POINT_4 = (5.81-tryx_offset, 3.09+3.0*0.03+3*1.3)
POINT_5 = (5.81-tryx_offset, 3.09+4.0*0.03+4*1.3)

# Point list: (5.57402,7.08162),(5.17163,7.09371),(4.00042,6.86198),(2.388,5.75416),(0.996634,3.76607)
# Point list: (5.63634,8.37777),(5.25668,8.31206),(3.53799,6.77407),(2.06884,5.34484),(1.03174,3.80271)


# CHK_PTS = [
#     [ POINT_S, POINT_C ], # scene 1
#     [ POINT_C,(3.50453,4.11369),(4.55161,5.41157),(5.19676,7.57945),POINT_1 ], # scene 2
#     [ POINT_C,(2.65163,2.93525),(4.34558,4.87455),POINT_2 ], # scene 3
#     [ POINT_C,(3.1, 3.57),POINT_3 ], # scene 4
#     [ POINT_C,(2.45023,2.59568),(4.19525,3.97616),POINT_4 ], # scene 5
#     [ POINT_C,(2.99331,3.31949),(4.56409,3.48977),POINT_5 ], # scene 6
#     [ POINT_C, POINT_S ], # scene 7
# ]

CHK_PTS = [
    # scene 0
    {
        'f_path': [ POINT_S, POINT_C ],
        'b_path': [ POINT_C, POINT_S ]
    },
    # scene 1
    {
        'f_path': [ POINT_C,(2.28189,2.8974),(3.85596,3.98624),POINT_1 ], 
        'b_path': [ POINT_1,(3.85596,3.98624),(2.28189,2.8974),POINT_C ]
    },
    # scene 2
    {
        'f_path': [ POINT_C,(2.32721,2.85522),(4.03223,4.21992),POINT_2 ], 
        'b_path': [ POINT_2,(4.03223,4.21992),(2.32721,2.85522),POINT_C ]
    },
    # scene 3
    {
        'f_path': [ POINT_C,(3.1, 3.57), POINT_3 ], 
        'b_path': [ POINT_3,(3.1, 3.57), POINT_C ]
    },
    # scene 4
    {
        'f_path': [ POINT_C,(2.65163,2.93525),(4.34558,4.87455),POINT_4 ], 
        'b_path': [ POINT_4,(5.17163,7.09371),(4.00042,6.86198),(2.388,5.75416),POINT_D ]
    },
    # scene 5
    {
        'f_path': [ POINT_D,(2.34632,5.8606),(4.7244,7.8566),POINT_5 ], 
        'b_path': [ (5.31206,8.40092),(4.7244,7.8566),(2.34632,5.8606),POINT_D ]
        # 'b_path': [ POINT_5,(5.37012,8.38554),(4.0548,7.06129),(2.07452,5.55428),POINT_D ]
    }
]

class PurePursuitConfig: # Default setting of MATCH_RED
    spline_label = "cubic"
    spline_type = PurePursuitData.CUBIC
    raw_pts = []
    point_density = 1000

    velocity_z_kP = 5.0
    velocity_z_kI = 0.01
    velocity_z_ilimit = 1.0
    velocity_z_max = 3.0
    target_z = -pi/2

    lookahead_distance = 0.0
    vector_policy = PurePursuitData.CURVATURE_DEPENDENT_TWIST_WITH_TANGENTIAL_PID
    # vector_policy = PurePursuitData.TANGENTIAL_PID_TWIST
    velocity_shift_kP = 6.0
    velocity_shift_kD = 0.01 # unstable!
    curvature_penalty_kP = 0.4

    stop_type = PurePursuitData.STOP_PID
    stop_pid_radius = 2.0

    field_color = MATCH_RED

    def __init__(self, spline_label, raw_pts):
        self.spline_label = spline_label
        if spline_label == "cubic":
            self.spline_type = PurePursuitData.CUBIC
        elif spline_label == "bezier":
            self.spline_type = PurePursuitData.BEZIER
        self.raw_pts = raw_pts
        self.chk_pts = [Point(x=point[0], y=point[1]) for point in self.raw_pts]

    def setFieldColor(self, color):
        if color not in [MATCH_RED, MATCH_BLUE]:
            return
        if self.field_color != color:
            self.knotsFlipX()
            self.field_color = color

    def knotsFlipX(self):
        self.chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in self.raw_pts]
        self.target_z = -self.target_z

    def goalConstructor(self, speed=2.0, radius=1.0, stop_min_speed=0.5, velocity_shift_kP=6.0, curvature_penalty_kP=0.4):
        pure_pursuit_data = PurePursuitData()
        pure_pursuit_data.label = self.spline_label
        pure_pursuit_data.spline_type = self.spline_type

        pure_pursuit_data.knots = self.chk_pts
        pure_pursuit_data.point_density = self.point_density
        
        pure_pursuit_data.velocity_xy_magnitude = speed

        pure_pursuit_data.velocity_z_kP = self.velocity_z_kP
        pure_pursuit_data.velocity_z_kI = self.velocity_z_kI
        pure_pursuit_data.velocity_z_ilimit = self.velocity_z_ilimit
        
        pure_pursuit_data.velocity_z_max = self.velocity_z_max
        pure_pursuit_data.target_z = self.target_z

        pure_pursuit_data.lookahead_distance = self.lookahead_distance
        pure_pursuit_data.vector_policy = self.vector_policy
        pure_pursuit_data.velocity_shift_kP = velocity_shift_kP
        pure_pursuit_data.velocity_shift_kD = self.velocity_shift_kD
        pure_pursuit_data.curvature_penalty_kP = curvature_penalty_kP

        pure_pursuit_data.stop_type = self.stop_type
        pure_pursuit_data.stop_radius = radius
        pure_pursuit_data.stop_deadzone = 0.05
        pure_pursuit_data.stop_min_speed = stop_min_speed

        goal = SwitchModeGoal(target_mode=SwitchModeGoal().PURE_PURSUIT, pure_pursuit_data=pure_pursuit_data)
        return goal

class PurePidConfig: # Default setting of MATCH_RED
    label = "pure_pid"

    velocity_z_max = 1.5
    velocity_z_kP = 3.0
    velocity_z_kI = 0.0
    velocity_z_kD = 0.1

    kP = 0.0
    kI = 0.0
    kD = 0.0
    integral_absMax = 0.5

    field_color = MATCH_RED

    def __init__(self, target_pos, target_z):
        self.target_x = target_pos[0]
        self.target_y = target_pos[1]
        self.target_z = target_z

    def setFieldColor(self, color):
        if color not in [MATCH_RED, MATCH_BLUE]:
            return
        if self.field_color != color:
            self.positionFlipX()
            self.field_color = color

    def positionFlipX(self):
        self.target_x = 13.3 - self.target_x
        self.target_z = -self.target_z

    def goalConstructor(self, speed, kP, kI, kD, PonE=True):
        pure_pid_data = PurePidData()
        pure_pid_data.label = self.label

        pure_pid_data.target_x = self.target_x
        pure_pid_data.target_y = self.target_y
        pure_pid_data.target_z = self.target_z

        pure_pid_data.velocity_xy_magnitude = speed

        pure_pid_data.velocity_z_max = self.velocity_z_max
        pure_pid_data.velocity_z_kP = self.velocity_z_kP
        pure_pid_data.velocity_z_kI = self.velocity_z_kI
        pure_pid_data.velocity_z_kD = self.velocity_z_kD

        pure_pid_data.kP = kP
        pure_pid_data.kI = kI
        pure_pid_data.kD = kD
        pure_pid_data.integral_absMax = self.integral_absMax

        pure_pid_data.deadzone_x = 0.02
        pure_pid_data.deadzone_y = 0.02
        pure_pid_data.deadzone_z = 0.0

        pure_pid_data.proportional_on_error = PonE

        goal = SwitchModeGoal(target_mode=SwitchModeGoal().PURE_PID, pure_pid_data=pure_pid_data)
        return goal

class TryspotBreakTrigger:
    x_min = 5.10
    x_max = 6.65
    y_min = 0.00
    y_max = 10.0
    thres = 0.90

    def getTriggers(self, color):
        if color == MATCH_RED:
            return (self.x_min, self.x_max, self.y_min, self.y_max, self.thres)
        elif color == MATCH_BLUE:
            return (13.3-self.x_min, 13.3-self.x_max, self.y_min, self.y_max, self.thres)
        else: 
            return None

scene0_f_cfg = PurePursuitConfig("cubic", CHK_PTS[0]['f_path'])
scene1_f_cfg = PurePursuitConfig("cubic", CHK_PTS[1]['f_path'])
scene2_f_cfg = PurePursuitConfig("cubic", CHK_PTS[2]['f_path'])
scene3_f_cfg = PurePursuitConfig("cubic", CHK_PTS[3]['f_path'])
scene4_f_cfg = PurePursuitConfig("cubic", CHK_PTS[4]['f_path'])
scene5_f_cfg = PurePursuitConfig("cubic", CHK_PTS[5]['f_path'])

try_cfg = TryspotBreakTrigger()
# wait_cfg = 
scene0_s_cfg = PurePidConfig(POINT_C, -pi/2)
scene1_s_cfg = PurePidConfig(POINT_1, -pi/2)
scene2_s_cfg = PurePidConfig(POINT_2, -pi/2)
scene3_s_cfg = PurePidConfig(POINT_3, -pi/2)
scene4_s_cfg = PurePidConfig(POINT_4, -pi/2)
scene5_s_cfg = PurePidConfig(POINT_5, -pi/2)
scene6_s_cfg = PurePidConfig(POINT_S, -pi/2)
pointC_cfg = PurePidConfig(POINT_C, -pi/2)
pointD_cfg = PurePidConfig(POINT_D, -2.10)

scene0_b_cfg = PurePursuitConfig("cubic", CHK_PTS[0]['b_path'])
scene1_b_cfg = PurePursuitConfig("cubic", CHK_PTS[1]['b_path'])
scene2_b_cfg = PurePursuitConfig("cubic", CHK_PTS[2]['b_path'])
scene3_b_cfg = PurePursuitConfig("cubic", CHK_PTS[3]['b_path'])
scene4_b_cfg = PurePursuitConfig("cubic", CHK_PTS[4]['b_path'])
scene5_b_cfg = PurePursuitConfig("cubic", CHK_PTS[5]['b_path'])
