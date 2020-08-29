#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import time

def odom_cb(odom):
    if max(odom.pose.covariance) >= 1000:
        # detected pose error
        data = time.time() % 2 == 0
        INVERT_PUBLISHER.publish(data=data)
    else:
        INVERT_PUBLISHER.publish(data=False)

def team_color_topic(invert = False):
    if LAUNCH_TEAM == ROBOT_TR1:
        if (LAUNCH_COLOR == MATCH_RED) != invert: # equality xor inversion
            return "/io_board1/io_0/set_state"
        if LAUNCH_COLOR == MATCH_BLUE: # equality xor inversion
            return "/io_board1/io_1/set_state"
    if LAUNCH_TEAM == ROBOT_TR2:
        if (LAUNCH_COLOR == MATCH_RED) != invert: # equality xor inversion
            return "/io_board1/io_6/set_state"
        if LAUNCH_COLOR == MATCH_BLUE: # equality xor inversion
            return "/io_board1/io_7/set_state"

    raise Exception("Unknown team/color")

if __name__ == "__main__":
    LAUNCH_COLOR = phrase_color_from_launch()
    LAUNCH_TEAM = phrase_team_from_launch()
    chassis_odom_topic = rospy.get_param("~chassis_odom_topic", "chassis_odom")
    rospy.Subscriber(chassis_odom_topic, Odometry, odom_cb)

    TEAM_PUBLISHER = rospy.Publisher(team_color_topic(), Bool, queue_size=1)
    INVERT_PUBLISHER = rospy.Publisher(team_color_topic(invert=True), Bool, queue_size=1)

    TEAM_PUBLISHER.publish(data=True)
    INVERT_PUBLISHER.publish(data=False)

    # TODO publish team color to PS4

    rospy.spin()
