#!/usr/bin/env python
from math import pi
from geometry_msgs.msg import *
from m2_move_base.msg import *

POINT_S = (0.5, 9.5)
POINT_C = (1.1, 1.1)
POINT_1 = (5.6, 3.09+0.5*0.03+0*1.3)
POINT_2 = (5.6, 3.09+1.0*0.03+1*1.3)
POINT_3 = (5.6, 3.09+2.0*0.03+2*1.3)
POINT_4 = (5.6, 3.09+3.0*0.03+3*1.3)
POINT_5 = (5.6, 3.09+4.0*0.03+4*1.3)

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
        'b_path': [ POINT_4,(4.34558,4.87455),(2.65163,2.93525), POINT_C ]
    },
    # scene 5
    {
        'f_path': [ POINT_C,(3.50453,4.11369),(4.55161,5.41157),(5.19676,7.57945),POINT_5 ], 
        'b_path': [ POINT_5,(5.19676,7.57945),(4.55161,5.41157),(3.50453,4.11369), POINT_C ]
    }
]

class PurePidConfig: # Default setting of MATCH_RED
    label = "pure_pid"

    velocity_z_max = 1.0
    velocity_z_kP = 2.0

    kP = 0.0
    kI = 0.0
    kD = 0.0
    integral_absMax = 100.0

    def __init__(self, target_pos, target_z):
        self.target_x = target_pos[0]
        self.target_y = target_pos[1]
        self.target_z = target_z

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

        pure_pid_data.kP = kP
        pure_pid_data.kI = kI
        pure_pid_data.kD = kD
        pure_pid_data.integral_absMax = self.integral_absMax

        pure_pid_data.proportional_on_error = PonE

        goal = SwitchModeGoal(target_mode=SwitchModeGoal().PURE_PID, pure_pid_data=pure_pid_data)
        return goal

class PurePursuitConfig: # Default setting of MATCH_RED
    spline_label = "cubic"
    spline_type = PurePursuitData.CUBIC
    raw_pts = []
    point_density = 1000

    velocity_z_kP = 3.0
    velocity_z_max = 2.0
    target_z = -pi/2

    lookahead_distance = 0.0
    vector_policy = PurePursuitData.CURVATURE_DEPENDENT_TWIST_WITH_TANGENTIAL_PID
    # vector_policy = PurePursuitData.TANGENTIAL_PID_TWIST
    velocity_shift_kP = 6.0
    velocity_shift_kD = 0.0 # unstable!
    curvature_penalty_kP = 0.4

    stop_type = PurePursuitData.STOP_PID
    stop_pid_integral_abs_max = 0.0
    # stop_pid_radius = 2.0
    # stop_kP = 2.0
    # stop_kD = 0.5

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

    def goalConstructor(self, pure_pid = False, speed=2.0, radius=1.0, stop_kI=0.0, stop_kD=0.0, velocity_shift_kP=6.0, curvature_penalty_kP=0.4):
        print(pure_pid, speed, radius, stop_kI, stop_kD, velocity_shift_kP, curvature_penalty_kP)
        pure_pursuit_data = PurePursuitData()
        pure_pursuit_data.label = self.spline_label
        pure_pursuit_data.spline_type = self.spline_type

        if(pure_pid):
          pure_pursuit_data.knots = self.chk_pts[:-1]
        else:
          pure_pursuit_data.knots = self.chk_pts
        pure_pursuit_data.point_density = self.point_density
        
        pure_pursuit_data.velocity_xy_magnitude = speed

        pure_pursuit_data.velocity_z_kP = self.velocity_z_kP
        pure_pursuit_data.velocity_z_max = self.velocity_z_max
        pure_pursuit_data.target_z = self.target_z

        pure_pursuit_data.lookahead_distance = self.lookahead_distance
        pure_pursuit_data.vector_policy = self.vector_policy
        pure_pursuit_data.velocity_shift_kP = velocity_shift_kP
        pure_pursuit_data.velocity_shift_kD = self.velocity_shift_kD
        pure_pursuit_data.curvature_penalty_kP = curvature_penalty_kP

        pure_pursuit_data.stop_type = self.stop_type
        pure_pursuit_data.stop_pid_integral_abs_max = self.stop_pid_integral_abs_max
        pure_pursuit_data.stop_pid_radius = radius
        pure_pursuit_data.stop_pid_integral_abs_max = 2 * radius
        pure_pursuit_data.stop_kP = speed/radius
        pure_pursuit_data.stop_kI = stop_kI
        pure_pursuit_data.stop_kD = stop_kD

        goal = SwitchModeGoal(target_mode=SwitchModeGoal().PURE_PURSUIT, pure_pursuit_data=pure_pursuit_data)
        return goal

# pursuit_0_f = PurePursuitConfig("cubic", CHK_PTS[0]['f_path'])
# pursuit_1_f = PurePursuitConfig("cubic", CHK_PTS[1]['f_path'])
# pursuit_2_f = PurePursuitConfig("cubic", CHK_PTS[2]['f_path'])
# pursuit_3_f = PurePursuitConfig("cubic", CHK_PTS[3]['f_path'])
# pursuit_4_f = PurePursuitConfig("cubic", CHK_PTS[4]['f_path'])
# pursuit_5_f = PurePursuitConfig("cubic", CHK_PTS[5]['f_path'])

# pursuit_0_b = PurePursuitConfig("cubic", CHK_PTS[0]['b_path'])
# pursuit_1_b = PurePursuitConfig("cubic", CHK_PTS[1]['b_path'])
# pursuit_2_b = PurePursuitConfig("cubic", CHK_PTS[2]['b_path'])
# pursuit_3_b = PurePursuitConfig("cubic", CHK_PTS[3]['b_path'])
# pursuit_4_b = PurePursuitConfig("cubic", CHK_PTS[4]['b_path'])
# pursuit_5_b = PurePursuitConfig("cubic", CHK_PTS[5]['b_path'])

# pid_0_f = PurePidConfig(CHK_PTS[0]['f_path'][-1])
# pid_1_f = PurePidConfig(CHK_PTS[1]['f_path'][-1])
# pid_2_f = PurePidConfig(CHK_PTS[2]['f_path'][-1])
# pid_3_f = PurePidConfig(CHK_PTS[3]['f_path'][-1])
# pid_4_f = PurePidConfig(CHK_PTS[4]['f_path'][-1])
# pid_5_f = PurePidConfig(CHK_PTS[5]['f_path'][-1])

# pid_0_b = PurePidConfig(CHK_PTS[0]['b_path'][-1])
# pid_1_b = PurePidConfig(CHK_PTS[1]['b_path'][-1])
# pid_2_b = PurePidConfig(CHK_PTS[2]['b_path'][-1])
# pid_3_b = PurePidConfig(CHK_PTS[3]['b_path'][-1])
# pid_4_b = PurePidConfig(CHK_PTS[4]['b_path'][-1])
# pid_5_b = PurePidConfig(CHK_PTS[5]['b_path'][-1])