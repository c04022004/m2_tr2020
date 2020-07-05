#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from m2_ps4.msg import Ps4Data
from std_msgs.msg import Bool

direct = Twist()
max_linear_speed = 3.0
max_rotational_speed = 1.5

old_data = Ps4Data()

def ps4_cb(ps4_data): # update ps4 data
    global direct
    direct.linear.y = max_linear_speed * ps4_data.hat_ly
    direct.linear.x = max_linear_speed * ps4_data.hat_lx * -1
    direct.angular.z = max_rotational_speed * ps4_data.hat_rx

    global old_data
    if ps4_data.triangle and not old_data.triangle: # dpad up
        io_pub.publish(1)
    if ps4_data.cross and not old_data.cross: # dpad down
        io_pub.publish(0)
    old_data = ps4_data

rospy.init_node('ps4_vel_controller')
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
ps4_sub = rospy.Subscriber('input/ps4_data', Ps4Data, ps4_cb)

io_pub = rospy.Publisher('io_2/set_state', Bool, queue_size=1)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    vel_pub.publish(direct)
    rate.sleep()

