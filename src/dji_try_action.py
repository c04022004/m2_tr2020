#!/usr/bin/env python3
import time, sys, threading
import rospy, actionlib
from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3
from m2_dji_can.srv import *
from m2_tr2020.msg import *
from actionlib_msgs.msg import GoalStatus

NUM_MOTORS = 1
motor0_pos = 0
motor0_current = 0

exec_event = threading.Event()

def get_position_cb(pos, motor_id):
    global motor0_pos
    if motor_id == 0:
        motor0_pos = pos.data


def rospy_shutdown_cb():
    rospy.logwarn("shutdown request received")
    if _as.is_active():
        _as.set_preempted()
    set_enable_state_srv(0, False)
    exec_event.set()

def as_preempt_cb():
    rospy.logwarn("dji_server goal preempt request")
    if _as.is_active():
        _as.set_preempted()
        set_enable_state_srv(0, False)
    exec_event.set()

def as_check_preempted():
    if not _as.is_active() :
        rospy.logwarn("shutdown request received, or dji_server was preempted. returning...")
        rospy.logwarn("_as.is_active(): %d"%(_as.is_active()))
        rospy.logwarn("_as.is_preempt_requested: %d"%(_as.is_preempt_requested()))
        set_enable_state_srv(0, False)
        exec_event.set()
        return True
    return False

def execute_cb(goal):
    start_time = time.time()
    exec_event.clear()
    
    # Init dji motor stuffs
    # result = set_offset_srv(0, motor0_pos) # Set offset as current encoder reading
    # result = set_current_limit_srv(0, 14745) # ~18A
    result = set_current_limit_srv(0, 12288) # 15A
    result = activate_p_mode_srv(0)

    if (goal.scene_id == 0):
        result = set_enable_state_srv(0, False)
        if as_check_preempted(): return
    elif (goal.scene_id == 1): ## Full procedure for auto (=3+4)
        # Make sure the slider is unblocked
        io_pub_slider.publish(1)
         # The actual try motion
        dji_m0_p_setpoint_pub.publish(-55000)
        result = set_enable_state_srv(0, True)
        while motor0_pos>-55000+1000 and not rospy.is_shutdown():
            if as_check_preempted(): return
        exec_event.wait(0.25) # Wait for PID to pull back (softer landing?)
        if as_check_preempted(): return

        # Let the metal structure fall
        result = set_enable_state_srv(0, False)
        exec_event.wait(0.25)
        if as_check_preempted(): return

        # Then retract back to the starting position
        dji_m0_p_setpoint_pub.publish(-12000)
        result = set_enable_state_srv(0, True)
        while motor0_pos<-12000-1000 and not rospy.is_shutdown():
            if as_check_preempted(): return
        exec_event.wait(0.7) # Wait for PID to pull back (softer landing?)
        # result = set_enable_state_srv(0, False)
        if as_check_preempted(): return
    elif (goal.scene_id == 2):
        # Raise a little bit to push the ball aside
        dji_m0_p_setpoint_pub.publish(-6500)
        result = set_enable_state_srv(0, True)
    elif (goal.scene_id == 3):
        # Make sure the slider is unblocked
        io_pub_slider.publish(1)
        # The actual try motion
        dji_m0_p_setpoint_pub.publish(-55000)
        result = set_enable_state_srv(0, True)
        while motor0_pos>-55000+1000 and not rospy.is_shutdown():
            if as_check_preempted(): return
        result = set_enable_state_srv(0, False)
    elif (goal.scene_id == 4):
        # Make sure the slider is unblocked
        io_pub_slider.publish(1)
        # Then retract back to the starting position
        dji_m0_p_setpoint_pub.publish(-15000)
        result = set_enable_state_srv(0, True)
        while motor0_pos<-15000-1000 and not rospy.is_shutdown():
            if as_check_preempted(): return
        result = set_enable_state_srv(0, False)
    else:
        rospy.logwarn("No matching goal number (%d)"%goal.scene_id)
        return

    if as_check_preempted(): return
    rospy.loginfo("dji action done. time=%f"%(time.time()-start_time))
    _as.set_succeeded(TryResult())

# ========== Main ==========

rospy.init_node("dji_try_server")
rate = rospy.Rate(100)

while not rospy.is_shutdown():
    try:
        rospy.wait_for_service('/dji/set_offset', 0.1)
        rospy.wait_for_service('/dji/set_current_limit', 0.1)
        rospy.wait_for_service('/dji/activate_p_mode', 0.1)
        rospy.wait_for_service('/dji/set_enable_state', 0.1)
        break
    except rospy.ROSException:
        rospy.logwarn_throttle(1,"Waiting for DJI related services")

for i in range(NUM_MOTORS):
    rospy.Subscriber("/dji/m%d/p_feedback" % i, Int32, get_position_cb, i, queue_size = 1)

set_offset_srv = rospy.ServiceProxy('/dji/set_offset', DJISetInt)
set_current_limit_srv = rospy.ServiceProxy('/dji/set_current_limit', DJISetInt)
activate_p_mode_srv = rospy.ServiceProxy('/dji/activate_p_mode', DJITrigger)
set_enable_state_srv = rospy.ServiceProxy('/dji/set_enable_state', DJISetBool)

dji_m0_p_setpoint_pub = rospy.Publisher("/dji/m0/p_setpoint", Int32, queue_size=10)
io_pub_slider = rospy.Publisher('io_board1/io_0/set_state', Bool, queue_size=1)

_action_name = "dji_try_server"
_as = actionlib.SimpleActionServer(_action_name, TryAction, execute_cb=execute_cb, auto_start=False)
time.sleep(0.1)
_as.start()
rospy.loginfo("dji_try_server started with all services ready!")
rospy.on_shutdown(rospy_shutdown_cb)
_as.register_preempt_callback(as_preempt_cb)