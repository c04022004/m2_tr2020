#!/usr/bin/env python
import rospy, time
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Trigger, SetBool
from m2_chassis_utils.msg import ChannelTwist
from m2_ps4.msg import Ps4Data, RgbTime
from m2_ps4.srv import SetRgb, SetRgbRequest
from std_msgs.msg import Bool
from m2_tr2020.msg import *
import numpy as np
import chassis_control

direct = ChannelTwist()
direct.channel = ChannelTwist.CONTROLLER
max_linear_speed = 1.5
max_rotational_speed = 1.2

old_data = Ps4Data()

# Select robot type
robot_sel = None
ROBOT_N   = 0
ROBOT_TR1 = 1
ROBOT_TR2 = 2

# Match field color
match_color = None
MATCH_NONE = 0
MATCH_RED  = 1
MATCH_BLUE = 2

# Define PS4 LED colors, 0.1s duration
BRIGHT_RED = (255,0,0,0.1)
DIM_RED = (50,0,0,0.1)
BRIGHT_Blue = (0,0,255,0.1)
DIM_Blue = (0,0,50,0.1)

# Saving slider position for TR2
slider_pos = False

# State of the ps4 handler
control_mode = None
MANUAL = 3
SEMI_AUTO = 4

def unblock_try():
    global slider_pos
    if robot_type == ROBOT_TR2:
        io_pub_slider.publish(1)
        if slider_pos != True:
            time.sleep(1.0)
            slider_pos = True

def update_led():
    rgb_req = SetRgbRequest(False, [])
    if match_color == MATCH_RED:
        rgb_req.rgb_sequence.append(RgbTime(*BRIGHT_RED))
        if control_mode == MANUAL:
            rgb_req.rgb_sequence.append(RgbTime(*DIM_RED))
    if match_color == MATCH_BLUE:
        rgb_req.rgb_sequence.append(RgbTime(*BRIGHT_BLUE))
        if control_mode == MANUAL:
            rgb_req.rgb_sequence.append(RgbTime(*DIM_BLUE))
    try:
        ps4_led_srv(rgb_req)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr_throttle("/set_led call failed")

def ps4_cb(ps4_data): # update ps4 data
    global direct,old_data,control_mode
    if ps4_data.l1 and ps4_data.r1:
        if ps4_data.share and not old_data.share:
            try_call_motors(False)
        if ps4_data.options and not old_data.options:
            try_call_motors(True)
    elif ps4_data.l1 and not old_data.l1:
        control_mode = MANUAL
        orientation_helper.stop_z()
        update_led()
        tr_cancel_pub.publish(GoalID())
        sw_cancel_pub.publish(GoalID())
        dji_cancel_pub.publish(GoalID())
        io_pub_slider.publish(1)
    elif ps4_data.r1 and not old_data.r1:
        control_mode = SEMI_AUTO
        update_led()

    if control_mode == MANUAL:
        # learnt the max speed of 1:15 motors the hard way, 4ms^-1 will burn the motor board (no longer true in 2020)
        # 3 ms^-1 linear + 1.2 rads^-1 almost max out human reaction
        twist = Twist()
        twist.linear.x  = np.copysign(np.abs(ps4_data.hat_lx)**1.75*max_linear_speed, ps4_data.hat_lx*-1)
        twist.linear.y  = np.copysign(np.abs(ps4_data.hat_ly)**1.75*max_linear_speed, ps4_data.hat_ly)
        twist.angular.z = ps4_data.hat_rx*max_rotational_speed
        vel_magnitude = np.hypot(twist.linear.x, twist.linear.y)
        if np.isclose(twist.angular.z,0.0):
            orientation_helper.stop_z()
        if vel_magnitude > max_linear_speed:
            twist.linear.x = twist.linear.x/vel_magnitude*max_linear_speed
            twist.linear.y = twist.linear.y/vel_magnitude*max_linear_speed
        fix_theta_vel = orientation_helper.compensate(twist)
        local_vel = kmt_helper.kmt_local2world(fix_theta_vel)

        global direct
        direct = ChannelTwist()
        direct.channel = ChannelTwist.CONTROLLER
        direct.linear = local_vel.linear
        direct.angular = local_vel.angular
        vel_pub.publish(direct)

        # Change ds4 button layout according to robot
        if robot_type == ROBOT_TR1:
            # try latch release/retract (io_7)
            if ps4_data.triangle and not old_data.triangle: # release/try
                io_pub_latch.publish(1)
            if ps4_data.cross and not old_data.cross: # retract
                io_pub_latch.publish(0)
        elif robot_type == ROBOT_TR2:
            # try slider release/retract (io_0)
            if ps4_data.triangle and not old_data.triangle: # pushing up
                io_pub_slider.publish(1)
            if ps4_data.cross and not old_data.cross: # pushing down
                io_pub_slider.publish(0)
            if ps4_data.square and not old_data.square: # dji_try
                unblock_try()
                goal = TryActionGoal()
                goal.goal.scene_id = 3
                dji_try_pub.publish(goal)
            if ps4_data.circle and not old_data.circle: # dji_up
                unblock_try()
                goal = TryActionGoal()
                goal.goal.scene_id = 4
                dji_try_pub.publish(goal)

    elif control_mode == SEMI_AUTO:
        if ps4_data.share and not old_data.share: # scene0/go wait 1st ball
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 0
            fulltask_pub.publish(goal)
        if ps4_data.options and not old_data.options: # tryspot1
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 1
            fulltask_pub.publish(goal)
        if ps4_data.triangle and not old_data.triangle: # tryspot2
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 2
            fulltask_pub.publish(goal)
        if ps4_data.circle and not old_data.circle: # tryspot3
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 3
            fulltask_pub.publish(goal)
        if ps4_data.cross and not old_data.cross: # tryspot4
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 4
            fulltask_pub.publish(goal)
        if ps4_data.square and not old_data.square: # tryspot5
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 5
            fulltask_pub.publish(goal)
        if (ps4_data.dpad_y == -1) and not old_data.dpad_y: # pointC
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 7
            fulltask_pub.publish(goal)
        if (ps4_data.dpad_y ==  1) and not old_data.dpad_y: # pointD
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 8
            fulltask_pub.publish(goal)
        if (ps4_data.dpad_x == -1) and not old_data.dpad_x: # back to start zone
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 6
            fulltask_pub.publish(goal)
    old_data = ps4_data

def try_call_motors(set_bool):
    global motor_srvs
    for i in range(4):
        try:
            rospy.wait_for_service('/motor_base_%d/set_enable_state'%i, timeout=0.1)
            motor_srvs[i](set_bool)
        except rospy.ROSException:
            rospy.logwarn('/motor_base_%d/set_enable_state timed out!'%i)

rospy.init_node('ps4_vel_controller')
vel_pub = rospy.Publisher('/chan_cmd_vel', ChannelTwist, queue_size = 1)
tr_cancel_pub = rospy.Publisher('/tr_server/cancel', GoalID, queue_size = 1)
sw_cancel_pub = rospy.Publisher('/Switch/cancel', GoalID, queue_size = 1)
dji_cancel_pub = rospy.Publisher('/dji_try_server/cancel', GoalID, queue_size = 1)
ps4_sub = rospy.Subscriber('input/ps4_data', Ps4Data, ps4_cb)
ps4_led_srv = rospy.ServiceProxy('/set_led', SetRgb)

# tr1/pneumatic
io_pub_latch = rospy.Publisher('io_board1/io_7/set_state', Bool, queue_size=1)
# tr2/djimotor
io_pub_slider = rospy.Publisher('io_board1/io_0/set_state', Bool, queue_size=1)
dji_try_pub = rospy.Publisher('/dji_try_server/goal', TryActionGoal, queue_size=1)

motor_srvs = [None for i in range(4)]
for i in range(4):
    motor_srvs[i] = rospy.ServiceProxy('/motor_base_%d/set_enable_state'%i, SetBool)
fulltask_pub = rospy.Publisher('/tr_server/goal', FulltaskActionGoal, queue_size=1)

kmt_helper = chassis_control.FrameTranslation()
orientation_helper = chassis_control.RotationCompesation()

color = rospy.get_param("~color", "red")
if color == "red":
    match_color = MATCH_RED
elif color == "blue":
    match_color = MATCH_BLUE
else:
    match_color = MATCH_RED
    rospy.logwarn("No valid color specified, defaulting to RED")
update_led()

team = rospy.get_param("~team", "rx")
if team == "rx":
    robot_type = ROBOT_TR1
elif team == "rtx":
    robot_type = ROBOT_TR2
else:
    rospy.logerr("No valid team specified, quitting")
    exit(1)

rospy.spin()

