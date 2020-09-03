import rospy

# Select robot type
ROBOT_NONE = 0
ROBOT_TR1  = 1
ROBOT_TR2  = 2

# Match field color
MATCH_NONE = 0
MATCH_RED  = 1
MATCH_BLUE = 2

def phrase_color_from_launch():
    color = rospy.get_param("~color", "red")
    if color == "red":
        rospy.loginfo("%s started with MATCH_RED"%rospy.get_name())
        return MATCH_RED
    elif color == "blue":
        rospy.loginfo("%s started with MATCH_BLUE"%rospy.get_name())
        return MATCH_BLUE
    else:
        rospy.logerr("No valid color specified, aborting")
        return MATCH_NONE

def phrase_team_from_launch():
    team = rospy.get_param("~team", "rx")
    if team == "rtx":
        rospy.loginfo("%s started with ROBOT_TR1"%rospy.get_name())
        return ROBOT_TR1
    elif team == "rx":
        rospy.loginfo("%s started with ROBOT_TR2"%rospy.get_name())
        return ROBOT_TR2
    else:
        rospy.logerr("No valid team specified, are you using fake robot?")
        return ROBOT_NONE