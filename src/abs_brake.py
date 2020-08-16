#!/usr/bin/env python
import rospy, time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from m2_chassis_utils.msg import ChannelTwist
import numpy as np
import chassis_control

# Motor enable state
prev_braking = False
is_braking = False
motor_en = True

# Decay coeff
coeff = 0.4

def brake_cb(msg):
    global is_braking
    if msg.data and not is_braking:
        rospy.loginfo("Start braking...")
        orientation_helper.stop_z()
        # orientation_helper.set_z(-np.pi/2)
    is_braking = msg.data

def odom_cb(odom_msg):
    if is_braking:
        twist = Twist()
        if np.hypot(twist.linear.x,twist.linear.y) > 0.75: # else zero vel
            twist.linear.x  = odom_msg.twist.twist.linear.x*coeff
            twist.linear.y  = odom_msg.twist.twist.linear.y*coeff
            twist.angular.z = odom_msg.twist.twist.angular.z*coeff*0.1

        fix_theta_vel = orientation_helper.compensate(twist)
        local_vel = kmt_helper.kmt_world2local(fix_theta_vel)

        direct = ChannelTwist()
        direct.channel = ChannelTwist.EMERGENCY
        direct.linear = local_vel.linear
        direct.angular = local_vel.angular
        vel_pub.publish(direct)

def try_call_motors(set_bool):
    global motor_srvs
    for i in range(4):
        try:
            rospy.wait_for_service('/motor_base_%d/set_enable_state'%i, timeout=0.1)
            motor_srvs[i](set_bool)
        except rospy.ROSException:
            rospy.logwarn('/motor_base_%d/set_enable_state timed out!'%i)

rospy.init_node('abs_braking')
rospy.loginfo("abs_braking mode: exponential decay %.2f"%coeff)

abs_sub = rospy.Subscriber('/abs/break', Bool, brake_cb)

kmt_helper = chassis_control.FrameTranslation()
orientation_helper = chassis_control.RotationCompesation(max_z_vel=3.0, kFF=0.0, kP=6.0)
vel_pub = rospy.Publisher('/chan_cmd_vel', ChannelTwist, queue_size = 1)

motor_srvs = [None for i in range(4)]
for i in range(4):
    motor_srvs[i] = rospy.ServiceProxy('/motor_base_%d/set_enable_state'%i, SetBool)

odom_sub = rospy.Subscriber("chassis_odom",Odometry, odom_cb)

rospy.spin()

# Real Anti-lock Braking is not possible:
# Epos4 only allow locking when the not moving
# The fastest enable call needs 0.1s to complete
# rate = rospy.Rate(10)
# while not rospy.is_shutdown():
#     if is_braking:
#         rospy.loginfo_throttle(0.5,"abs_braking ON!")
#         # Forcing the vel to be zero
#         vel_pub.publish(direct)
#         # Alternating enable and release, req 0.1s for each motor
#         motor_en = not motor_en
#         try_call_motors(motor_en)
#     rate.sleep()
