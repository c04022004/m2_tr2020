#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from m2_ps4.msg import Ps4Data

direct = Twist()
max_linear_speed = 4.0
max_rotational_speed = 1.5

def ps4_cb(ps4_data): # update ps4 data
    global direct
    direct.linear.y = max_linear_speed * ps4_data.hat_ly
    direct.linear.x = max_linear_speed * ps4_data.hat_lx * -1
    direct.angular.z = max_rotational_speed * ps4_data.hat_rx

rospy.init_node('ps4_vel_controller')
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
ps4_sub = rospy.Subscriber('input/ps4_data', Ps4Data, ps4_cb)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    vel_pub.publish(direct)
    rate.sleep()