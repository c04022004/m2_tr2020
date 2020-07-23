#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from m2_chassis_utils.msg import ChannelTwist
from m2_ps4.msg import Ps4Data
from std_msgs.msg import Bool
from m2_tr2020.msg import FulltaskActionGoal

direct = ChannelTwist()
direct.channel = ChannelTwist.CONTROLLER
max_linear_speed = 1.5
max_rotational_speed = 1.2

old_data = Ps4Data()

def ps4_cb(ps4_data): # update ps4 data
    
    global direct,old_data
    if ps4_data.l1:
        direct.linear.y = max_linear_speed * ps4_data.hat_ly
        direct.linear.x = max_linear_speed * ps4_data.hat_lx * -1
        direct.angular.z = max_rotational_speed * ps4_data.hat_rx
        vel_pub.publish(direct)
        tr_cancel_pub.publish(GoalID())
        sw_cancel_pub.publish(GoalID())
        # try latch release/retract (io_7)
        if ps4_data.triangle and not old_data.triangle: # release/try
            io_pub.publish(1)
        if ps4_data.cross and not old_data.cross: # retract
            io_pub.publish(0)
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
    old_data = ps4_data

rospy.init_node('ps4_vel_controller')
vel_pub = rospy.Publisher('/chan_cmd_vel', ChannelTwist, queue_size = 1)
tr_cancel_pub = rospy.Publisher('/tr_server/cancel', GoalID, queue_size = 1)
sw_cancel_pub = rospy.Publisher('/Switch/cancel', GoalID, queue_size = 1)
ps4_sub = rospy.Subscriber('input/ps4_data', Ps4Data, ps4_cb)

io_pub = rospy.Publisher('io_board1/io_7/set_state', Bool, queue_size=1)
fulltask_pub = rospy.Publisher('/tr_server/goal', FulltaskActionGoal, queue_size=1)

rospy.spin()

