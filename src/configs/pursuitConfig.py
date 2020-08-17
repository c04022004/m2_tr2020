#!/usr/bin/env python
from math import pi
from geometry_msgs.msg import *
from m2_move_base.msg import *
from .fieldConfig import *
from .commConfig import *

MAX_SPEED = 4.5

POINT_S = (0.5, 9.5)
POINT_0 = (0.8, 0.48)   # 1st ball pass
POINT_C = (0.8, 1.02)   # 
POINT_D = (1.1, 3.5)    # z=-2.3
POINT_E = (0.6, 4.95)   # z=-2.1
# end-pt x 6150 - 300(half robot) - 40(protector) 
# end-pt y 3090 + 30(white line)  + 1300(offset)
tryx_offset = 0.05 # reserve for overshoot
POINT_1 = (5.81-tryx_offset, 3.09+0.5*0.03+0*1.3)
POINT_2 = (5.81-tryx_offset, 3.09+1.0*0.03+1*1.3)
POINT_3 = (5.81-tryx_offset, 3.09+2.0*0.03+2*1.3)
POINT_4 = (5.81-tryx_offset, 3.09+3.0*0.03+3*1.3)
POINT_5 = (5.81-tryx_offset, 3.09+4.0*0.03+4*1.3)

CHK_PTS = [
    # scene 0
    {
        'curve': "cubic",
        'f_path': [ POINT_S, POINT_0 ],
        'b_path': None,
    },
    # scene 1
    {
        'curve': "cubic",
        'f_path': [ POINT_D,(3.34892,3.71807),POINT_1 ], 
        # 'b_path': [ POINT_1,(4.30239,3.87097),(2.89165,3.77514),POINT_D ], # TS2 -> POINT_D, via TZ2
        'b_path': [ POINT_1,(3.53595,4.60383),POINT_E ], # TS2 -> POINT_E, via TZ2
    },
    # scene 2
    {
        'curve': "cubic",
        # 'f_path': [ POINT_C,(2.32721,2.85522),(4.03223,4.21992),POINT_2 ], # POINT_C -> POINT_2, via TZ2
        # 'f_path': [ POINT_D,(2.89165,3.77514),(4.66508,4.29096),POINT_2 ], # POINT_D -> POINT_2, via TZ2
        'f_path': [ POINT_D,(3.34892,3.71807),POINT_2 ], # POINT_D -> POINT_2, via TZ2
        # 'b_path': [ POINT_2,(4.03223,4.21992),(2.32721,2.85522),POINT_C ], # TS2 -> POINT_C, via TZ2
        # 'b_path': [ POINT_2,(4.66508,4.29096),(2.89165,3.77514),POINT_D ], # TS2 -> POINT_D, via TZ2
        'b_path': [ POINT_2,(3.34892,3.71807),POINT_D ], # TS2 -> POINT_D, via TZ2
        # 'b_path': [ POINT_2,(3.35334,4.9082),POINT_E ], # TS2 -> POINT_E, via TZ1
    },
    # scene 3
    {
        'curve': "cubic",
        'f_path': [ POINT_C,(3.1, 3.57), POINT_3 ], 
        # 'b_path': [ POINT_3,(3.1, 3.57), POINT_C ], # TS3 -> POINT_C, via TZ2
        'b_path': [ POINT_3,(3.74106,4.17264), POINT_D], # TS3 -> POINT_D, via TZ2
    },
    # scene 4
    {
        'curve': "cubic",
        # 'f_path': [ POINT_C,(2.65163,2.93525),(4.34558,4.87455),POINT_4 ], # POINT_C -> TS4, via TZ2
        # 'f_path': [ POINT_D,(2.3880,5.75416),(4.00042,6.86198),(5.17163,7.09371),POINT_4 ], # POINT_D -> TS4, via TZ1
        'f_path': [ POINT_E,(3.33397,6.47919),POINT_4 ], # POINT_E -> TS4, via TZ1
        # 'b_path': [ POINT_4,(5.17163,7.09371),(4.00042,6.86198),(2.3880,5.75416),POINT_D ],
        'b_path': [ POINT_4,(3.33397,6.47919),POINT_E ], # TS4 -> POINT_E, via TZ1
    },
    # scene 5
    {
        'curve': "cubic",
        # 'f_path': [ POINT_D,(2.34632,5.8606),(4.7244,7.8566),POINT_5 ], # POINT_D -> TS5, via TZ1
        'f_path': [ POINT_E,(3.33397,6.47919),POINT_5 ], # POINT_E -> TS5, via TZ1
        # 'b_path': [ (5.31206,8.40092),(4.7244,7.8566),(2.34632,5.8606),POINT_D ], # TS5 -> POINT_E, via TZ1
        'b_path': [ POINT_5,(3.33397,6.47919),POINT_E ], # TS5 -> POINT_E, via TZ1
    },
    # scene 6
    {
        'curve': "cubic",
        'f_path': [ POINT_C, POINT_S ],
        'b_path': None,
    },
]

class PurePursuitConfig: # Default setting of MATCH_RED
    spline_label = "cubic"
    spline_type = PurePursuitData.CUBIC
    raw_pts = []
    point_density = 1000

    velocity_z_kP = 3.0
    velocity_z_kI = 1.0
    velocity_z_ilimit = 1.5
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
        pure_pursuit_data.stop_min_speed = stop_min_speed
        pure_pursuit_data.stop_deadzone = 0.05

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

    def goalConstructor(self, speed, kP, kI, kD, PonE=True, accel_lim=10.0):
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
        pure_pid_data.deadzone_z = 0.017 # 1deg

        pure_pid_data.accel_limit = accel_lim
        pure_pid_data.force_compensate = True
        pure_pid_data.force_compensate_kP = 1.0

        pure_pid_data.proportional_on_error = PonE

        goal = SwitchModeGoal(target_mode=SwitchModeGoal().PURE_PID, pure_pid_data=pure_pid_data)
        return goal

class BreakTrigger:
    field_color = MATCH_RED

    def __init__(self, x_min=None, x_max=None, y_min=None, y_max=None, eta=None, thres=0.98):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.eta   = eta
        self.thres = thres

    def setFieldColor(self, color):
        if color not in [MATCH_RED, MATCH_BLUE]:
            return
        if self.field_color != color:
            self.positionFlipX()
            self.field_color = color
    
    def positionFlipX(self):
        self.x_min = 13.3 - self.x_min
        self.x_max = 13.3 - self.x_max
        self.x_min,self.x_max = self.x_max,self.x_min

    def getTriggers(self):
        return (self.x_min, self.x_max, self.y_min, self.y_max, self.eta, self.thres)

cfg = {}

# Setup paths from CHK_PTS
for i in range(7):
    if CHK_PTS[i]['f_path'] != None:
        cfg['scene%d_f'%i]  = PurePursuitConfig(CHK_PTS[i]['curve'], CHK_PTS[i]['f_path'])
        cfg['scene%d_fs'%i] = PurePidConfig(CHK_PTS[i]['f_path'][-1], -pi/2)
    if CHK_PTS[i]['b_path'] != None:
        cfg['scene%d_b'%i]  = PurePursuitConfig(CHK_PTS[i]['curve'], CHK_PTS[i]['b_path'])
        cfg['scene%d_bs'%i] = PurePidConfig(CHK_PTS[i]['b_path'][-1], -2.10 if CHK_PTS[i]['b_path'][-1] == POINT_D else -pi/2)
    
# for i in range(1,6):
#     cfg['scene%d_f'%i].target_z -= 0.25 # slightly slanted orientation
# Meeting significant motor limit here, must use all 4 motors (and less air resistance)
cfg['scene3_f'].target_z -= 0.25
cfg['scene3_b'].target_z -= 0.25
cfg['scene4_f'].target_z -= 0.25
cfg['scene4_b'].target_z -= 0.25
cfg['scene5_f'].target_z -= 0.25
cfg['scene5_b'].target_z -= 0.25

cfg['pointC_s'] = PurePidConfig(POINT_C, -pi/2)
cfg['pointD_s'] = PurePidConfig(POINT_D, -2.10)

# Setup try tolerances (x,y) 
tryx_tol = abs(0.1/2)
tryy_tol = abs(0.1/2)
for i in range(1,6):
    pt = globals()['POINT_%d'%i]
    cfg['ts%d_trig'%i] = BreakTrigger(x_min=pt[0]-tryx_tol,x_max=pt[0]+tryx_tol,y_min=pt[1]-tryy_tol,y_max=pt[1]+tryy_tol,thres=0.98)

# Setup trigger to pre-notify PR for passing rugby
cfg['rec_sc0_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_0[0],'dest_y':POINT_0[1],'kP':3.0,'kD':2.5,'vel':MAX_SPEED*0.40,'dz':0.05},thres=0.98)
cfg['rec_ptC_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_C[0],'dest_y':POINT_C[1],'kP':3.0,'kD':4.0,'vel':MAX_SPEED*0.60,'dz':0.05},thres=0.98)
cfg['rec_ptD_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_D[0],'dest_y':POINT_D[1],'kP':3.0,'kD':4.0,'vel':MAX_SPEED*0.60,'dz':0.05},thres=0.98)
cfg['rec_ptE_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_E[0],'dest_y':POINT_E[1],'kP':3.0,'kD':4.0,'vel':MAX_SPEED*0.60,'dz':0.05},thres=0.98)

# Setup mode transition criteria
cfg['try_pid_trig'] = BreakTrigger(x_min=5.1,x_max=6.65,y_min=0.0,y_max=10.0,thres=0.90)
cfg['rec_pid_trig'] = BreakTrigger(x_min=0.0,x_max=1.50,y_min=0.0,y_max=10.0,thres=0.90)
cfg['mode_trig']    = BreakTrigger(thres=0.90)
cfg['default_trig'] = BreakTrigger(thres=0.99)

# Setup the path sequence
# In general:
# velocity_shift_kP = 1.0 when super-straight, 6.0 when curved
# curvature_penalty_kP = 0.1 when super-straight, upto 0.8 when curved
# stopping pid: speed*0.45, 3.0, 0.0001, 3.0 OR
# stopping pid: speed*0.60, 3.0, 0.0001, 4.0
try0_param = [  {'hook_func' : [{'call_pr':[START_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,},
                {'hook_func' : [{'ball_guard':None}],
                 'cfg_name'  : 'scene0_f',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.5, 'stop_min_speed':0.75,
                                'velocity_shift_kP':1.0, 'curvature_penalty_kP':0.0},
                 'trig_name' : 'mode_trig',
                 'log_msg'   : "start running for receiving pos",},
                {'hook_func' : [{'ball_guard':None}],
                 'cfg_name'  : 'scene0_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.4, 'kP':3.0, 'kI':0.0001, 'kD':2.5, 'accel_lim':5.5},
                 'trig_name' : 'rec_sc0_trig',
                 'log_msg'   : "breaking stage before receiving pos",},
                None, None,
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try1_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene1_f',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 1",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene1_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.45, 'kP':3.0, 'kI':0.0001, 'kD':3.0, 'accel_lim':5.0},
                 'trig_name' : 'ts1_trig',
                 'log_msg'   : "breaking stage before Try Spot 1",},
                {'hook_func' : [{'do_try':None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 1",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_SLANT]}],
                 'cfg_name'  : 'scene1_b',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene1_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.6, 'kP':3.0, 'kI':0.0001, 'kD':4.0, 'accel_lim':5.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try2_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene2_f',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 2",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene2_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.45, 'kP':3.0, 'kI':0.0001, 'kD':3.0},
                 'trig_name' : 'ts2_trig',
                 'log_msg'   : "breaking stage before Try Spot 2",},
                {'hook_func' : [{"do_try":None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 2",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_SLANT]}],
                 'cfg_name'  : 'scene2_b',
                 'cfg_param' : {'speed':MAX_SPEED*0.9, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene2_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.6, 'kP':3.0, 'kI':0.0001, 'kD':4.0, 'accel_lim':5.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try3_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene3_f',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':1.0, 'curvature_penalty_kP':0.1},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 3",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene3_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.45, 'kP':1.5, 'kI':0.0001, 'kD':3.0, 'accel_lim':5.0},
                 'trig_name' : 'ts3_trig',
                 'log_msg'   : "breaking stage before Try Spot 3",},
                {'hook_func' : [{'do_try':None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 3",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_DIRECT]}],
                 'cfg_name'  : 'scene3_b',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':1.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene3_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.6, 'kP':3.0, 'kI':0.0001, 'kD':4.0, 'accel_lim':5.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try4_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene4_f',
                 'cfg_param' : {'speed':MAX_SPEED*0.9, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.8},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 4",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene4_fs',
                 'cfg_param' : {'speed':2.1, 'kP':3.0, 'kI':0.0001, 'kD':2.5},
                 'trig_name' : 'ts4_trig',
                 'log_msg'   : "breaking stage before Try Spot 4",},
                {'hook_func' : [{'do_try':None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 4",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_SLANT]}],
                 'cfg_name'  : 'scene4_b',
                 'cfg_param' : {'speed':MAX_SPEED*0.9, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene4_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.6, 'kP':3.0, 'kI':0.0001, 'kD':4.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try5_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene5_f',
                 'cfg_param' : {'speed':MAX_SPEED*0.9, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.8},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 5",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene5_fs',
                 'cfg_param' : {'speed':2.1, 'kP':3.0, 'kI':0.0001, 'kD':2.5},
                 'trig_name' : 'ts5_trig',
                 'log_msg'   : "breaking stage before Try Spot 5",},
                {'hook_func' : [{'do_try':None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 5",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_SLANT]}],
                 'cfg_name'  : 'scene5_b',
                 'cfg_param' : {'speed':MAX_SPEED*0.9, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene5_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.6, 'kP':3.0, 'kI':0.0001, 'kD':4.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

for i in range(1,6):
    try_param = globals()['try%d_param'%i]
    if cfg[try_param[3]['cfg_name']].raw_pts[-1] == POINT_C:
        try_param[4]['trig_name'] = "rec_ptC_trig"
    if cfg[try_param[3]['cfg_name']].raw_pts[-1] == POINT_D:
        try_param[4]['trig_name'] = "rec_ptD_trig"
    if cfg[try_param[3]['cfg_name']].raw_pts[-1] == POINT_E:
        try_param[4]['trig_name'] = "rec_ptE_trig"

try6_param = [  {'hook_func' : [{'ball_guard':None}],
                 'cfg_name'  : 'scene6_f',
                 'cfg_param' : {'speed':MAX_SPEED*0.6, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : "mode_trig",
                 'log_msg'   : "start running for TRSZ",},
                {'hook_func' : [{'ball_guard':None}],
                 'cfg_name'  : 'scene6_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.5, 'kP':3.0, 'kI':0.0001, 'kD':2.5},
                 'trig_name' : "default_trig",
                 'log_msg'   : "breaking stage before TRSZ",},
                None, None, None,
                {'hook_func' : [{'ball_guard':None}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try7_param = [  {'hook_func' : None,
                 'cfg_name'  : None,
                 'log_msg'   : "start running for PointC",},
                {'hook_func' : None,
                 'cfg_name'  : 'pointC_s',
                 'cfg_param' : {'speed':MAX_SPEED*0.6, 'kP':3.0, 'kI':0.0001, 'kD':2.5},
                 'trig_name' : "default_trig",
                 'log_msg'   : "breaking stage before PointC",},
                None, None, None,
                {'hook_func' : [{'ball_guard':None}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try8_param = [  {'hook_func' : None,
                 'cfg_name'  : None,
                 'log_msg'   : "start running for PointD",},
                {'hook_func' : None,
                 'cfg_name'  : 'pointD_s',
                 'cfg_param' : {'speed':MAX_SPEED*0.5, 'kP':3.0, 'kI':0.0001, 'kD':2.5},
                 'trig_name' : "default_trig",
                 'log_msg'   : "breaking stage before PointD",},
                None, None, None,
                {'hook_func' : [{'ball_guard':None}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

