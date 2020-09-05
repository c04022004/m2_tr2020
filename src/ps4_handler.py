#!/usr/bin/env python
import rospy, time, threading
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Trigger, SetBool, SetBoolRequest
from m2_chassis_utils.msg import ChannelTwist
from m2_ps4.msg import Ps4Data, RgbTime, FfTime
from m2_ps4.srv import SetRgb, SetRgbRequest, SetFf, SetFfRequest
from std_msgs.msg import Bool
from m2_tr2020.msg import *
import numpy as np
from configs.fieldConfig import *
import chassis_control

old_data = Ps4Data()

# ROS launch config
match_color = None
robot_type = None

# Define PS4 LED colors, 0.1s duration
BRIGHT_RED = (255,0,0,0.1)
DIM_RED = (50,0,0,0.1)
BRIGHT_BLUE = (0,0,255,0.1)
DIM_BLUE = (0,0,50,0.1)
BRIGHT_WHITE = (255,255,255,0.1)
DIM_WHITE = (50,50,50,0.1)

# Saving slider position for TR2
slider_pos = False

# State of the ps4 handler
control_mode = None
FULL_MANUAL = 3
ASSISTED_MANUAL =4
SEMI_AUTO = 5
NO_MOTOR = 6

# Motor enable state
motor_en = True

# Odom position
odom_pos = None
is_offset = False

def slider_cb(msg):
    global slider_pos
    slider_pos = msg.data

def unblock_try():
    if robot_type == ROBOT_TR2:
        io_pub_slider.publish(1)
        if slider_pos != True:
            rospy.sleep(0.6)
            return slider_pos
    return True

def update_led():
    rgb_req = SetRgbRequest(False, [])
    if match_color == MATCH_RED:
        rgb_req.rgb_sequence.append(RgbTime(*BRIGHT_RED))
        if control_mode == ASSISTED_MANUAL:
            rgb_req.rgb_sequence.append(RgbTime(*DIM_RED))
        elif control_mode == NO_MOTOR:
            rgb_req.rgb_sequence.append(RgbTime(*DIM_BLUE))
    if match_color == MATCH_BLUE:
        rgb_req.rgb_sequence.append(RgbTime(*BRIGHT_BLUE))
        if control_mode == ASSISTED_MANUAL:
            rgb_req.rgb_sequence.append(RgbTime(*DIM_BLUE))
        elif control_mode == NO_MOTOR:
            rgb_req.rgb_sequence.append(RgbTime(*DIM_RED))
    if control_mode == FULL_MANUAL:
        rgb_req = SetRgbRequest(False, [])
        rgb_req.rgb_sequence.append(RgbTime(*BRIGHT_WHITE))
        rgb_req.rgb_sequence.append(RgbTime(*DIM_WHITE))
    try:
        ps4_led_srv(rgb_req)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr_throttle(10,"/set_led call failed")

def cancel_all_action():
    tr_cancel_pub.publish(GoalID())
    sw_cancel_pub.publish(GoalID())
    if robot_type == ROBOT_TR2:
        dji_cancel_pub.publish(GoalID())

def odom_cb(odom_msg):
    global odom_pos
    if odom_pos == None:
        odom_pos = {}
    odom_pos["x"] = odom_msg.pose.pose.position.x
    odom_pos["y"] = odom_msg.pose.pose.position.y
    quaternion = (
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w)
    odom_pos["z"] = transformations.euler_from_quaternion(quaternion)[2]

def ps4_cb(ps4_data): # update ps4 data
    global old_data,control_mode,motor_en
    if ps4_data.l2 and ps4_data.r2:
        if ps4_data.share and not old_data.share:
            try_call_motors(False)
            control_mode = NO_MOTOR
            update_led()
        if ps4_data.options and not old_data.options:
            try_call_motors(True)
            control_mode = FULL_MANUAL
            update_led()
    elif ps4_data.l1 and not old_data.l1:
        if control_mode!= ASSISTED_MANUAL and orientation_helper is not None:
            orientation_helper.stop_z()
        control_mode = ASSISTED_MANUAL
        try_call_motors(True)
        update_led()
        cancel_all_action()
    elif ps4_data.r1 and not old_data.r1:
        control_mode = SEMI_AUTO
        cancel_all_action()
        try_call_motors(True)
        update_led()

    # Velocity related calculations
    direct = ChannelTwist()
    if control_mode == FULL_MANUAL:
        local_vel = Twist()
        local_vel.linear.x = np.copysign(np.abs(ps4_data.hat_lx)**1.7*max_linear_speed, -ps4_data.hat_lx)
        local_vel.linear.y = np.copysign(np.abs(ps4_data.hat_ly)**1.7*max_linear_speed, ps4_data.hat_ly)
        local_vel.angular.z = ps4_data.hat_rx*max_rotational_speed
        direct.channel = ChannelTwist.EMERGENCY
        direct.linear = local_vel.linear
        direct.angular = local_vel.angular
        vel_pub.publish(direct)
    elif control_mode == ASSISTED_MANUAL:
        # learnt the max speed of 1:15 motors the hard way, 4ms^-1 will burn the motor board (no longer true in 2020)
        # 3 ms^-1 linear + 1.5 rads^-1 almost max out human reaction
        global_vel = Twist()
        if match_color == MATCH_RED:
            # non-linear control
            global_vel.linear.x = np.copysign(np.abs(ps4_data.hat_ly)**1.7*max_linear_speed, ps4_data.hat_ly)
            global_vel.linear.y = np.copysign(np.abs(ps4_data.hat_lx)**1.7*max_linear_speed, ps4_data.hat_lx)
            global_vel.angular.z = ps4_data.hat_rx*max_rotational_speed
        elif match_color == MATCH_BLUE:
            # remap the direction
            global_vel.linear.x = np.copysign(np.abs(ps4_data.hat_ly)**1.7*max_linear_speed, ps4_data.hat_ly*-1)
            global_vel.linear.y = np.copysign(np.abs(ps4_data.hat_lx)**1.7*max_linear_speed, ps4_data.hat_lx*-1)
            global_vel.angular.z = ps4_data.hat_rx*max_rotational_speed

        vel_magnitude = np.hypot(global_vel.linear.x, global_vel.linear.y)
        if np.isclose(global_vel.angular.z,0.0) and np.isclose(vel_magnitude,0.0):
            orientation_helper.stop_z()
        if vel_magnitude > max_linear_speed:
            global_vel.linear.x = global_vel.linear.x/vel_magnitude*max_linear_speed
            global_vel.linear.y = global_vel.linear.y/vel_magnitude*max_linear_speed
        fix_theta_vel = orientation_helper.compensate(global_vel)
        local_vel = kmt_helper.kmt_world2local(fix_theta_vel)

        # Publish velocity or brake
        if ps4_data.l1:
            abs_pub.publish(True)
        else:
            abs_pub.publish(False)
            direct.channel = ChannelTwist.CONTROLLER
            direct.linear = local_vel.linear
            direct.angular = local_vel.angular
            vel_pub.publish(direct)

    elif control_mode == SEMI_AUTO:
        # control offset at reciving position, vel fixed at 1.8ms^-1
        offset_speed = 2.0
        global_vel = Twist()
        ff_req = SetFfRequest([])
        if match_color == MATCH_RED:
            # non-linear control
            global_vel.linear.x = np.copysign(np.abs(ps4_data.hat_ly)**1.7*offset_speed, ps4_data.hat_ly)
            global_vel.linear.y = np.copysign(np.abs(ps4_data.hat_lx)**1.7*offset_speed, ps4_data.hat_lx)
            global_vel.angular.z = ps4_data.hat_rx*max_rotational_speed
            # limit the vel by odom_pos
            if odom_pos != None:
                if odom_pos["x"] > 1.57-0.6:
                    global_vel.linear.x = min(0.1,global_vel.linear.x)
                    ff_req = SetFfRequest([FfTime(64000,40000,0.1,0.0)])
                if odom_pos["x"] < 0.67:
                    global_vel.linear.x = max(-0.1,global_vel.linear.x)
                    ff_req = SetFfRequest([FfTime(64000,40000,0.1,0.0)])
        elif match_color == MATCH_BLUE:
            # remap the direction
            global_vel.linear.x = np.copysign(np.abs(ps4_data.hat_ly)**1.7*offset_speed, ps4_data.hat_ly*-1)
            global_vel.linear.y = np.copysign(np.abs(ps4_data.hat_lx)**1.7*offset_speed, ps4_data.hat_lx*-1)
            global_vel.angular.z = ps4_data.hat_rx*max_rotational_speed
            # limit the vel by odom_pos
            if odom_pos != None:
                if odom_pos["x"] > 13.3-0.67:
                    global_vel.linear.x = min(0.1,global_vel.linear.x)
                    ff_req = SetFfRequest([FfTime(64000,40000,0.1,0.0)])
                if odom_pos["x"] < 13.3-(1.57-0.60):
                    global_vel.linear.x = max(-0.1,global_vel.linear.x)
                    ff_req = SetFfRequest([FfTime(64000,40000,0.1,0.0)])

        vel_magnitude = np.hypot(global_vel.linear.x, global_vel.linear.y)
        if np.isclose(global_vel.angular.z,0.0) and np.isclose(vel_magnitude,0.0):
            orientation_helper.stop_z()
        if vel_magnitude > offset_speed:
            global_vel.linear.x = global_vel.linear.x/vel_magnitude*offset_speed
            global_vel.linear.y = global_vel.linear.y/vel_magnitude*offset_speed
        fix_theta_vel = orientation_helper.compensate(global_vel)
        local_vel = kmt_helper.kmt_world2local(fix_theta_vel)

        # Publish velocity if manual offset is non-zero
        global is_offset
        if not (np.isclose(global_vel.angular.z,0.0) and np.isclose(vel_magnitude,0.0)):
            is_offset = True
            direct.channel = ChannelTwist.CONTROLLER
            direct.linear = local_vel.linear
            direct.angular = local_vel.angular
            vel_pub.publish(direct)
            try:
                ps4_ff_srv(ff_req)
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr_throttle(10,"/set_ff call failed")
        elif is_offset:
            is_offset = False
            direct = ChannelTwist() # Stop moving
            direct.channel = ChannelTwist.CONTROLLER
            vel_pub.publish(direct)
            try:
                ps4_ff_srv(SetFfRequest([])) # Stop rumble
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr_throttle(10,"/set_ff call failed")

    # Try-related functions
    if control_mode == FULL_MANUAL or control_mode == ASSISTED_MANUAL:
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
                if unblock_try():
                    goal = TryActionGoal()
                    goal.goal.scene_id = 3
                    dji_try_pub.publish(goal)
            if ps4_data.circle and not old_data.circle: # dji_up
                if unblock_try():
                    goal = TryActionGoal()
                    goal.goal.scene_id = 4
                    dji_try_pub.publish(goal)

    elif control_mode == SEMI_AUTO:
        if not ps4_data.r1:
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
        else:
            if ps4_data.options and not old_data.options: # tryspot1
                goal = FulltaskActionGoal()
                goal.goal.scene_id = 11
                fulltask_pub.publish(goal)
            if ps4_data.triangle and not old_data.triangle: # tryspot2
                goal = FulltaskActionGoal()
                goal.goal.scene_id = 12
                fulltask_pub.publish(goal)
            if ps4_data.circle and not old_data.circle: # tryspot3
                goal = FulltaskActionGoal()
                goal.goal.scene_id = 13
                fulltask_pub.publish(goal)
            if ps4_data.cross and not old_data.cross: # tryspot4
                goal = FulltaskActionGoal()
                goal.goal.scene_id = 14
                fulltask_pub.publish(goal)
            if ps4_data.square and not old_data.square: # tryspot5
                goal = FulltaskActionGoal()
                goal.goal.scene_id = 15
                fulltask_pub.publish(goal)

        POINT_S = {'x': 0.5, 'y': 9.50}
        POINT_A = {'x': 0.8, 'y': 0.48}
        def dist_to_PT(pt, odom):
            dx = pt['x'] - odom['x']
            dy = pt['y'] - odom['y']
            return np.hypot(dx,dy)

        if (ps4_data.dpad_x == 1) and not old_data.dpad_x: # back to start zone
            if match_color == MATCH_RED:
                # back to start zone (TRSZ)
                goal = FulltaskActionGoal()
                goal.goal.scene_id = 6 # no pub cuz too dangerous
            elif match_color == MATCH_BLUE:
                # scene0/go wait 1st ball
                goal = FulltaskActionGoal()
                if dist_to_PT(POINT_S,odom_pos)<2.0:
                    goal.goal.scene_id = 0
                elif dist_to_PT(POINT_A,odom_pos)<4.0:
                    goal.goal.scene_id = 9
                fulltask_pub.publish(goal)
        if (ps4_data.dpad_x == -1) and not old_data.dpad_x: # back to start zone
            if match_color == MATCH_RED:
                # scene0/go wait 1st ball
                goal = FulltaskActionGoal()
                if dist_to_PT(POINT_S,odom_pos)<2.0:
                    goal.goal.scene_id = 0
                elif dist_to_PT(POINT_A,odom_pos)<4.0:
                    goal.goal.scene_id = 9
                fulltask_pub.publish(goal)
            elif match_color == MATCH_BLUE:
                # back to start zone (TRSZ)
                goal = FulltaskActionGoal()
                goal.goal.scene_id = 6 # no pub cuz too dangerous

    old_data = ps4_data

def try_call_motors(set_bool):
    global motor_srvs
    if robot_type not in [ROBOT_TR1, ROBOT_TR2]:
        return
    for i in range(4):
        try:
            rospy.wait_for_service('/motor_base_%d/set_enable_state'%i, timeout=0.1)
            motor_srvs[i](set_bool)
        except rospy.ROSException:
            rospy.logwarn('/motor_base_%d/set_enable_state timed out!'%i)

rospy.init_node('ps4_vel_controller')
max_linear_speed = rospy.get_param('~max_speed')
max_rotational_speed = 1.5
vel_pub = rospy.Publisher('/chan_cmd_vel', ChannelTwist, queue_size = 1)
tr_cancel_pub = rospy.Publisher('/tr_server/cancel', GoalID, queue_size = 1)
sw_cancel_pub = rospy.Publisher('/Switch/cancel', GoalID, queue_size = 1)
dji_cancel_pub = rospy.Publisher('/dji_try_server/cancel', GoalID, queue_size = 1)
odom_sub = rospy.Subscriber("chassis_odom",Odometry,odom_cb)
ps4_sub = rospy.Subscriber('input/ps4_data', Ps4Data, ps4_cb)
ps4_led_srv = rospy.ServiceProxy('/set_led', SetRgb)
ps4_ff_srv = rospy.ServiceProxy('/set_ff', SetFf)

# tr1/pneumatic
io_pub_latch = rospy.Publisher('io_board1/io_7/set_state', Bool, queue_size=1)
# tr2/djimotor
io_pub_slider = rospy.Publisher('io_board1/io_0/set_state', Bool, queue_size=1)
io_sub_slider = rospy.Subscriber('io_board1/io_3/get_state', Bool, slider_cb)
dji_try_pub = rospy.Publisher('/dji_try_server/goal', TryActionGoal, queue_size=1)

# need to launch a seperate abs_brake util
abs_pub = rospy.Publisher('/abs/break', Bool, queue_size=1)

motor_srvs = [None for i in range(4)]
for i in range(4):
    motor_srvs[i] = rospy.ServiceProxy('/motor_base_%d/set_enable_state'%i, SetBool)
fulltask_pub = rospy.Publisher('/tr_server/goal', FulltaskActionGoal, queue_size=1)

kmt_helper = chassis_control.FrameTranslation()
orientation_helper = chassis_control.RotationCompesation(max_z_vel=3.0, kFF=0.6, kP=4.0)

match_color = phrase_color_from_launch()
robot_type = phrase_team_from_launch()

if match_color == None:
    exit(1)

rospy.spin()

