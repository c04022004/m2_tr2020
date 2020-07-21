#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from m2_chassis_utils.msg import ChannelTwist
from m2_ps4.msg import Ps4Data
from std_msgs.msg import Bool

direct = ChannelTwist()
direct.channel = ChannelTwist.CONTROLLER
max_linear_speed = 1.5
max_rotational_speed = 1.2

old_data = Ps4Data()

def ps4_cb(ps4_data): # update ps4 data
    global direct
    if ps4_data.l1:
        direct.linear.y = max_linear_speed * ps4_data.hat_ly
        direct.linear.x = max_linear_speed * ps4_data.hat_lx * -1
        direct.angular.z = max_rotational_speed * ps4_data.hat_rx
        vel_pub.publish(direct)
        tr_cancel_pub.publish(GoalID())
        sw_cancel_pub.publish(GoalID())

    global old_data
    # try latch release/retract (io_7)
    if ps4_data.triangle and not old_data.triangle:
        io_pub.publish(1)
        io_pub.publish(1)
    if ps4_data.cross and not old_data.cross: # dpad down
    if ps4_data.cross and not old_data.cross:
        io_pub.publish(0)
        io_pub.publish(0)

rospy.init_node('ps4_vel_controller')
vel_pub = rospy.Publisher('/chan_cmd_vel', ChannelTwist, queue_size = 1)
tr_cancel_pub = rospy.Publisher('/tr_server/cancel', GoalID, queue_size = 1)
sw_cancel_pub = rospy.Publisher('/Switch/cancel', GoalID, queue_size = 1)
ps4_sub = rospy.Subscriber('input/ps4_data', Ps4Data, ps4_cb)

io_pub = rospy.Publisher('io_7/set_state', Bool, queue_size=1)

rospy.spin()

