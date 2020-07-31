#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Trigger, SetBool
from m2_chassis_utils.msg import ChannelTwist
from m2_ps4.msg import Ps4Data
from std_msgs.msg import Bool
from m2_tr2020.msg import *

direct = ChannelTwist()
direct.channel = ChannelTwist.CONTROLLER
max_linear_speed = 1.5
max_rotational_speed = 1.2

old_data = Ps4Data()
robot_sel = None

def ps4_cb(ps4_data): # update ps4 data
    
    global direct,old_data
    if ps4_data.l1 and ps4_data.r1:
        if ps4_data.share and not old_data.share:
            try_call_motors(False)
        if ps4_data.options and not old_data.options:
            try_call_motors(True)
    elif ps4_data.l1:
        direct.linear.y = max_linear_speed * ps4_data.hat_ly
        direct.linear.x = max_linear_speed * ps4_data.hat_lx * -1
        direct.angular.z = max_rotational_speed * ps4_data.hat_rx
        vel_pub.publish(direct)
        tr_cancel_pub.publish(GoalID())
        sw_cancel_pub.publish(GoalID())
        dji_cancel_pub.publish(GoalID())
        if robot_sel == 1:
            # try latch release/retract (io_7)
            if ps4_data.triangle and not old_data.triangle: # release/try
                io_pub_latch.publish(1)
            if ps4_data.cross and not old_data.cross: # retract
                io_pub_latch.publish(0)
        elif robot_sel == 2:
            # try slider release/retract (io_0)
            if ps4_data.triangle and not old_data.triangle: # pushing up
                io_pub_slider.publish(1)
            if ps4_data.cross and not old_data.cross: # retract
                io_pub_slider.publish(0)
            if ps4_data.square and not old_data.square: # dji_try
                goal = TryActionGoal()
                goal.goal.scene_id = 1
                dji_try_pub.publish(goal)
            if ps4_data.circle and not old_data.circle: # dji_try
                goal = TryActionGoal()
                goal.goal.scene_id = 2
                dji_try_pub.publish(goal)
    elif ps4_data.r1:
        # do_try in fulltask_server
        if ps4_data.share and not old_data.share:
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 6
            fulltask_pub.publish(goal)
        if ps4_data.options and not old_data.options:
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 0
            fulltask_pub.publish(goal)
        if ps4_data.triangle and not old_data.triangle:
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 2
            fulltask_pub.publish(goal)
        if ps4_data.circle and not old_data.circle:
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 3
            fulltask_pub.publish(goal)
        if ps4_data.cross and not old_data.cross:
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 4
            fulltask_pub.publish(goal)
        if ps4_data.square and not old_data.square:
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 5
            fulltask_pub.publish(goal)
        if (ps4_data.dpad_y == -1) and not old_data.dpad_y:
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 7
            fulltask_pub.publish(goal)
        if (ps4_data.dpad_y ==  1) and not old_data.dpad_y:
            goal = FulltaskActionGoal()
            goal.goal.scene_id = 8
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

# tr1/pneumatic
io_pub_latch = rospy.Publisher('io_board1/io_7/set_state', Bool, queue_size=1)
# tr2/djimotor
io_pub_slider = rospy.Publisher('io_board1/io_0/set_state', Bool, queue_size=1)
dji_try_pub = rospy.Publisher('/dji_try_server/goal', TryActionGoal, queue_size=1)

motor_srvs = [None for i in range(4)]
for i in range(4):
    motor_srvs[i] = rospy.ServiceProxy('/motor_base_%d/set_enable_state'%i, SetBool)
fulltask_pub = rospy.Publisher('/tr_server/goal', FulltaskActionGoal, queue_size=1)

team = rospy.get_param("~team")
if team == "rx":
    robot_sel = 1
elif team == "rtx":
    robot_sel = 2
else:
    rospy.logerr("Please specify a robot")
    exit(1)

rospy.spin()

