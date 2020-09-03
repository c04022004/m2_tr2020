#!/usr/bin/env python
from math import pi
from geometry_msgs.msg import *
from m2_move_base.msg import *
from .fieldConfig import *
from .commConfig import *

MAX_SPEED = 1.8

POINT_S = (0.5, 9.5)
POINT_A = (0.8, 0.48)   # 1st ball pass (to TS3)
POINT_B = (0.92, 3.2)   # 2nd ball pass (to TS2)
POINT_C = (1.12, 3.7)   # 3rd ball pass (to TS1)
POINT_D = (0.92, 4.5)   # 5th ball pass (to TS5
POINT_E = (0.92, 5.3)   # 4th ball pass (to TS4)

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
        'f_path': [ POINT_S, POINT_A ],
        'b_path': None,
    },
    # scene 1
    {
        'curve': "cubic",
        # POINT_C -> TS1, via TZ2 (mostly straight)
        'f_path': [ POINT_C,(3.33919,3.58903),POINT_1 ],
        # TS2 -> POINT_E, via TZ2 (curved)
        'b_path': [ POINT_1,(3.68049,4.56372),POINT_E ],
    },
    # scene 2
    {
        'curve': "cubic",
        # POINT_B -> TS2, via TZ2 (mostly straight)
        # (5.64966,4.43162),(3.40273,3.8522),(0.906773,3.2515)
        'f_path': [ POINT_B,(3.31654,3.69184),POINT_2 ],
        # TS2 -> POINT_C, via TZ2 (curved)
        'b_path': [ POINT_2,(3.31654,3.69184),POINT_C ],
    },
    # scene 3
    {
        'curve': "cubic",
        # always the first ball (mostly straight)
        'f_path': [ POINT_A,(3.1, 3.57), POINT_3 ],
        # TS3 -> POINT_B, via TZ2 (slightly curved)
        'b_path': [ POINT_3,(3.1, 3.57), POINT_B ],
    },
    # scene 4
    {
        'curve': "cubic",
        # POINT_E -> TS4, via TZ1 (mostly straight)
        'f_path': [ POINT_E,(3.37841,6.23819),POINT_4 ],
        # TS4 -> POINT_D, via TZ1 (curved)
        'b_path': [ POINT_4,(3.10521,6.32539),POINT_D ],
    },
    # scene 5
    {
        'curve': "cubic",
        # POINT_D -> TS5, via TZ1 (mostly straight)
        'f_path': [ POINT_D,(3.13137,6.26445),POINT_5 ],
        # TS5 -> POINT_E, via TZ1 (starting position retracted)
        'b_path': [ (POINT_5[0]-0.25,POINT_5[1]),(3.13137,6.26445),POINT_D ],
    },
    # scene 6
    {
        'curve': "cubic",
        'f_path': [ POINT_D, POINT_S ],
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
    velocity_z_max = 4.0
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

        pure_pid_data.deadzone_x = 0.005
        pure_pid_data.deadzone_y = 0.005
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
        if self.x_min!= None:
            self.x_min = 13.3 - self.x_min
        if self.x_max!= None:
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
        cfg['scene%d_bs'%i] = PurePidConfig(CHK_PTS[i]['b_path'][-1], -pi/2)

for i in range(1,6): # slightly slanted orientation
    if i in [1,3]:
        cfg['scene%d_f'%i].target_z -= 0.25
    if i in [4,5]:
        cfg['scene%d_f'%i].target_z -= 0.30
    if i in [2,3]:
        cfg['scene%d_b'%i].target_z -= 0.25
    if i in [1,4,5]:
        cfg['scene%d_b'%i].target_z -= 0.30

    if (cfg['scene%d_bs'%i].target_x, cfg['scene%d_bs'%i].target_y) == POINT_A:
        pass
    elif (cfg['scene%d_bs'%i].target_x, cfg['scene%d_bs'%i].target_y) == POINT_B:
        cfg['scene%d_bs'%i].target_z = -pi/2 -pi/6
    elif (cfg['scene%d_bs'%i].target_x, cfg['scene%d_bs'%i].target_y) == POINT_B:
        cfg['scene%d_bs'%i].target_z = -pi/2 -pi/6
    elif (cfg['scene%d_bs'%i].target_x, cfg['scene%d_bs'%i].target_y) == POINT_C:
        cfg['scene%d_bs'%i].target_z = -pi/2 -pi/5
    elif (cfg['scene%d_bs'%i].target_x, cfg['scene%d_bs'%i].target_y) == POINT_D:
        cfg['scene%d_bs'%i].target_z = -pi/2 -pi/5
    elif (cfg['scene%d_bs'%i].target_x, cfg['scene%d_bs'%i].target_y) == POINT_E:
        cfg['scene%d_bs'%i].target_z = -pi/2 -pi/4
# Meeting significant motor limit here, must use all 4 motors (and less air resistance)

# Setup try tolerances (x,y) 
tryx_tol = abs(0.1/2)
tryy_tol = abs(0.08/2)
for i in range(1,6):
    pt = globals()['POINT_%d'%i]
    cfg['ts%d_trig'%i] = BreakTrigger(x_min=pt[0]-tryx_tol,x_max=pt[0]+tryx_tol,y_min=pt[1]-tryy_tol,y_max=pt[1]+tryy_tol,thres=0.99)

# Setup mode transition criteria
cfg['try_pid_trig'] = BreakTrigger(x_min=5.1,x_max=6.65,y_min=0.0,y_max=10.0,thres=0.90)
cfg['rec_pid_trig'] = BreakTrigger(x_min=0.0,x_max=1.50,y_min=0.0,y_max=10.0,thres= 0.90)
cfg['mode_trig']    = BreakTrigger(thres=0.90)
cfg['default_trig'] = BreakTrigger(thres=0.99)

# Setup the path sequence
# In general:
# velocity_shift_kP = 2.0 when super-straight, 6.0 when curved
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
                 'trig_name' : 'rec_ptA_trig',
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
                 'cfg_param' : {'speed':MAX_SPEED*0.45, 'kP':2.0, 'kI':0.0001, 'kD':3.0, 'accel_lim':5.5},
                 'trig_name' : 'ts1_trig',
                 'log_msg'   : "breaking stage before Try Spot 1",},
                {'hook_func' : [{'do_try':None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 1",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_PT_E]}],
                 'cfg_name'  : 'scene1_b',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':8.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene1_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.5, 'kP':2.0, 'kI':0.0001, 'kD':4.0, 'accel_lim':5.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try2_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene2_f',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':2.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 2",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene2_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.45, 'kP':2.0, 'kI':0.0001, 'kD':3.0, 'accel_lim':5.5},
                 'trig_name' : 'ts2_trig',
                 'log_msg'   : "breaking stage before Try Spot 2",},
                {'hook_func' : [{"do_try":None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 2",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_PT_C]}],
                 'cfg_name'  : 'scene2_b',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene2_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.5, 'kP':2.0, 'kI':0.0001, 'kD':4.0, 'accel_lim':5.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try3_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene3_f',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':2.0, 'curvature_penalty_kP':0.1},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 3",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene3_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.45, 'kP':2.0, 'kI':0.001, 'kD':3.0, 'accel_lim':5.5},
                 'trig_name' : 'ts3_trig',
                 'log_msg'   : "breaking stage before Try Spot 3",},
                {'hook_func' : [{'do_try':None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 3",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_PT_B]}],
                 'cfg_name'  : 'scene3_b',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.1},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene3_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.5, 'kP':2.0, 'kI':0.0001, 'kD':4.0, 'accel_lim':5.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try4_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene4_f',
                 'cfg_param' : {'speed':MAX_SPEED*1.0, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':8.0, 'curvature_penalty_kP':0.4},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 4",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene4_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.45, 'kP':2.0, 'kI':0.0001, 'kD':3.0, 'accel_lim':5.5},
                 'trig_name' : 'ts4_trig',
                 'log_msg'   : "breaking stage before Try Spot 4",},
                {'hook_func' : [{'do_try':None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 4",},
                {'hook_func' : [{'comm_pr_try_done':[CAN_CLIP_PT_D]}],
                 'cfg_name'  : 'scene4_b',
                 'cfg_param' : {'speed':MAX_SPEED*0.9, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':8.0, 'curvature_penalty_kP':0.6},
                 'trig_name' : 'rec_pid_trig',
                 'log_msg'   : "back to receiving pos",},
                {'hook_func' : [{'hook4':None}],
                 'cfg_name'  : 'scene4_bs',
                 'trig_name' : 'default_trig',
                 'cfg_param' : {'speed':MAX_SPEED*0.5, 'kP':2.0, 'kI':0.0001, 'kD':4.0, 'accel_lim':5.0},
                 'log_msg'   : "pid into receiving pos",},
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

try5_param = [  {'hook_func' : [{'hook0':None}],
                 'cfg_name'  : 'scene5_f',
                 'cfg_param' : {'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_min_speed':0.75,
                                'velocity_shift_kP':8.0, 'curvature_penalty_kP':0.8},
                 'trig_name' : 'try_pid_trig',
                 'log_msg'   : "start running for Try Spot 5",},
                {'hook_func' : [{'hook1':None}],
                 'cfg_name'  : 'scene5_fs',
                 'cfg_param' : {'speed':MAX_SPEED*0.40, 'kP':2.0, 'kI':0.0001, 'kD':3.0, 'accel_lim':5.5},
                 'trig_name' : 'ts5_trig',
                 'log_msg'   : "breaking stage before Try Spot 5",},
                {'hook_func' : [{'do_try':None}],
                 'cfg_name'  : None,
                 'log_msg'   : "do_try for Try Spot 5",},
                None, None,
                {'hook_func' : [{'ball_guard':None},{'call_pr':[CAN_PASS_COMMAND]}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]

# Setup trigger to pre-notify PR for passing rugby
cfg['rec_ptA_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_A[0],'dest_y':POINT_A[1],'kP':3.0,'kD':2.5,'vel':MAX_SPEED*0.40,'dz':0.05},thres=0.98)
cfg['rec_ptB_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_B[0],'dest_y':POINT_B[1],'kP':2.0,'kD':4.0,'vel':MAX_SPEED*0.50,'dz':0.05},thres=0.98)
cfg['rec_ptC_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_C[0],'dest_y':POINT_C[1],'kP':2.0,'kD':4.0,'vel':MAX_SPEED*0.50,'dz':0.05},thres=0.98)
cfg['rec_ptD_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_D[0],'dest_y':POINT_D[1],'kP':2.0,'kD':4.0,'vel':MAX_SPEED*0.50,'dz':0.05},thres=0.98)
cfg['rec_ptE_trig'] = BreakTrigger(eta={'time':1.00,'dest_x':POINT_E[0],'dest_y':POINT_E[1],'kP':2.0,'kD':4.0,'vel':MAX_SPEED*0.50,'dz':0.05},thres=0.98)

for i in range(1,6):
    try_param = globals()['try%d_param'%i]
    if try_param[3] != None and cfg[try_param[3]['cfg_name']] != None:
        if cfg[try_param[3]['cfg_name']].raw_pts[-1] == POINT_A:
            try_param[4]['trig_name'] = "rec_ptA_trig"
        if cfg[try_param[3]['cfg_name']].raw_pts[-1] == POINT_B:
            try_param[4]['trig_name'] = "rec_ptB_trig"
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
                 'cfg_name'  : 'scene3_bs',
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
                 'cfg_name'  : 'scene1_bs',
                 'cfg_param' : {'speed':MAX_SPEED*0.5, 'kP':3.0, 'kI':0.0001, 'kD':2.5},
                 'trig_name' : "default_trig",
                 'log_msg'   : "breaking stage before PointD",},
                None, None, None,
                {'hook_func' : [{'ball_guard':None}],
                 'cfg_name'  : None,
                 'log_msg'   : None,}  ]