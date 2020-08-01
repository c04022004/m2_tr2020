import rospy

# Select robot type
robot_type = None
ROBOT_NONE = 0
ROBOT_TR1  = 1
ROBOT_TR2  = 2

# Match field color
match_color = None
MATCH_NONE = 0
MATCH_RED  = 1
MATCH_BLUE = 2

def phrase_color_from_launch():
    global match_color
    color = rospy.get_param("~color", "red")
    if color == "red":
        match_color = MATCH_RED
        rospy.loginfo("%s started with MATCH_RED"%rospy.get_name())
    elif color == "blue":
        match_color = MATCH_BLUE
        rospy.loginfo("%s started with MATCH_BLUE"%rospy.get_name())
    else:
        match_color = MATCH_NONE
        rospy.logwarn("No valid color specified, aborting")
        exit(1)

def phrase_team_from_launch():
    global robot_type
    team = rospy.get_param("~team", "rx")
    if team == "rx":
        robot_type = ROBOT_TR1
        rospy.loginfo("%s started with ROBOT_TR1"%rospy.get_name())
    elif team == "rtx":
        robot_type = ROBOT_TR2
        rospy.loginfo("%s started with ROBOT_TR2"%rospy.get_name())
    else:
        robot_type = ROBOT_NONE
        rospy.logerr("No valid team specified, are you using fake robot?")