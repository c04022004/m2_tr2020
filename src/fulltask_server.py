#!/usr/bin/env python
import rospy, actionlib, threading
from m2_move_base.msg import *
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from math import pi
from m2_tr2020.msg import *
import time
from actionlib_msgs.msg import GoalStatus
from configs.pursuitConfig import *
from configs.fieldConfig import *

match_color = None
robot_type = None

class FulltaskSceneHandler(object):

    def __init__(self):
        self.stop_srv = rospy.Service('stop_srv', Trigger, self.stop_cb)
        self.move_base_client = actionlib.SimpleActionClient('Switch', SwitchModeAction)
        self.move_base_client.wait_for_server()

        self.io_pub_latch = rospy.Publisher('io_board1/io_7/set_state', Bool, queue_size=1) # try latch release/retract (io_7)
        self.io_pub_slider = rospy.Publisher('io_board1/io_0/set_state', Bool, queue_size=1) # try slider release/retract (io_0)
        self.dji_client = actionlib.SimpleActionClient('dji_try_server', TryAction)
        # self.dji_client.wait_for_server()
        self.delayed_lifter_thread = None
        self.delayed_slider_thread = None
        self.try_event = threading.Event()

        self.cartop_status_pub = rospy.Publisher("cartop_status", Marker, queue_size = 1)
        self.omni_frame = rospy.get_param("~base_frame", "omni_base")

        self._action_name = "tr_server"
        self._as = actionlib.SimpleActionServer(self._action_name, FulltaskAction, execute_cb=self.execute_cb, auto_start=False)
        self.path_finish_event = threading.Event()
        self._as.register_preempt_callback(self.as_preempt_cb)
        self._as.start()
        rospy.on_shutdown(self.rospy_shutdown_cb)

        if match_color not in [MATCH_RED, MATCH_BLUE]:
            rospy.logerr("No field color selected, aborting!")
            exit(1)

    def stop_cb(self, req):
        self.path_finish_event.set() # set the finish event manually, usual for preempting a goal action
        # you can even set self.move_base_client to an invalid object such that it is uncallable in the remainder of that scene
        # The reverse action should be evemt.clear()
        return (True, "event set")
    
    def gen_intermediate_func(self, x_min=0.0, x_max=0.0, y_min=0.0, y_max=0.0, thres=0.99):
        def intermediate_func(msg):
            pose = msg.cur_pos.pose.pose
            if pose.position.x >= x_min and pose.position.x <= x_max and\
                pose.position.y >= y_min and pose.position.y <= y_max:
                self.path_finish_event.set()
            if (msg.progress > thres):
                self.path_finish_event.set() # python threading event
        return intermediate_func

    def rospy_shutdown_cb(self):
        rospy.logwarn("shutdown request received")
        if self._as.is_active():
            self.move_base_client.cancel_all_goals()
            self.dji_client.cancel_all_goals()
            self._as.set_preempted()
        self.path_finish_event.set()

    def move_base_client_done_cb(self, state, result):
        # from actionlib_msgs.msg import GoalStatus
        # State enum: http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        states = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"]
        rospy.logwarn("move_base_client goal done: State=%d %s"%(state, states[state]))
        self.path_finish_event.set()
    
    def as_state_decode(self, state, action_name):
        states = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"]
        rospy.logwarn("%s goal done: State=%d %s"%(action_name, state, states[state]))

    def as_preempt_cb(self): # only called when the goal is cancelled externally outisde of this node (e.g. publishing to tr_server/cancel)
        rospy.logwarn("fulltask goal preempt request")
        if self._as.is_active():
            self.move_base_client.cancel_all_goals()
            self.dji_client.cancel_all_goals()
            self._as.set_preempted()
        self.path_finish_event.set()

    def as_check_preempted(self):
        if not self._as.is_active() :
            rospy.logwarn("shutdown request received, or fulltask_server was preempted. returning...")
            rospy.logwarn("_as.is_active(): %d"%(self._as.is_active()))
            rospy.logwarn("_as.is_preempt_requested: %d"%(self._as.is_preempt_requested()))
            return True
        return False
    
    def hookPH(self):
        pass # Just a placeholder here
    
    def hook0(self):
        self.ball_guard()
        if robot_type == ROBOT_TR2:
            self.dji_client.send_goal(TryGoal(scene_id=2)) # Pre-lift the rugby

    def hook1(self):
        if robot_type == ROBOT_TR2:
             # Can have it delayed using threading
            self.slider_io(1) # slide up the rugby protector

    def hook2(self):
        pass

    def hook3(self):
        pass

    def hook4(self):
        if robot_type == ROBOT_TR2:
            self.dji_client.send_goal(TryGoal(scene_id=0))
            self.dji_client.wait_for_result()
            self.as_state_decode(self.dji_client.get_state(),"dji_try_server")
        self.ball_guard()

    def hook5(self):
        self.ball_guard()

    def do_try(self):
        if robot_type == ROBOT_NONE:
            rospy.logwarn("If you are not using fake_robot, go check your code!")
            rospy.logwarn("Default to a 1.5 second sleep when ROBOT_NONE is set!")
            # self.path_finish_event.wait(1.5)
            time.sleep(1.5)
            self.as_check_preempted()
            return
        elif robot_type == ROBOT_TR1:
            self.io_pub_latch.publish(1)
            self.path_finish_event.wait(0.75)
            if self.as_check_preempted(): return
            if self.delayed_lifter_thread != None and self.delayed_lifter_thread.isAlive():
                self.delayed_lifter_thread.cancel()
            self.delayed_lifter_thread = threading.Timer(0.1, self.ball_guard)
            self.delayed_lifter_thread.start()
        elif robot_type == ROBOT_TR2:
            self.io_pub_slider.publish(1)
            self.dji_client.send_goal_and_wait(TryGoal(scene_id=3))
            self.as_state_decode(self.dji_client.get_state(),"dji_try_server")
            self.try_event.clear()
            self.try_event.wait(0.5)

            self.dji_client.send_goal(TryGoal(scene_id=4))
            self.as_state_decode(self.dji_client.get_state(),"dji_try_server")
            # self.try_event.clear()
            # self.try_event.wait(0.7)

            if self.delayed_slider_thread != None and self.delayed_slider_thread.isAlive():
                self.delayed_slider_thread.cancel()
            # self.delayed_slider_thread = threading.Timer(1.0, self.ball_guard)
            # self.delayed_slider_thread.start()

    def latch_io(self, output=0):
        self.io_pub_latch.publish(output)

    def slider_io(self, output=0):
        self.io_pub_slider.publish(output)

    def ball_guard(self):
        if robot_type == ROBOT_TR1:
            self.latch_io(0) # retract
        elif robot_type == ROBOT_TR2:
            self.slider_io(0) # down

    def process_hooks(self, hook_list):
        for hook_dict in hook_list:
            for hook, arg in hook_dict.items():
                rospy.loginfo("Processing hook self.%s"%hook)
                try:
                    method = getattr(self, hook)
                    if arg != None:
                        method(*arg)
                    else:
                        method()
                except AttributeError as e:
                    rospy.logerr(e)
                    rospy.logerr("Class %s does not implement %s"%(self.__class__.__name__,hook))

    def scene_func(self, scene_num, params):
        rospy.loginfo("Fulltask scene %d start!"% scene_num)
        start_time = time.time()

        # --- Try sequence - stage0~4 ---
        for i in range(5):
            if params[i] != None:
                # Processing the hooks
                self.process_hooks(params[i]['hook_func'])
                # Start the pure_pursuit events
                if params[i]['cfg_name'] != None:
                    self.path_finish_event.clear()
                    self.move_base_client.send_goal(
                        cfg[params[i]['cfg_name']].goalConstructor(**params[i]['cfg_param']),
                        feedback_cb=self.gen_intermediate_func(*cfg[params[i]['trig_name']].getTriggers()),
                        done_cb=self.move_base_client_done_cb)
                    rospy.loginfo(params[i]['log_msg'])
                    self.path_finish_event.wait()
                    if self.as_check_preempted(): return

        # --- Try sequence - stage5 ---
        # Processing the hooks
        self.process_hooks(params[5]['hook_func'])
        self.path_finish_event.clear()
        if params[5]['cfg_name'] != None:
            rospy.logerr("Not suppose to run more path here!")

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def sample_scene(self):
        rospy.loginfo("move done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())
        pass

    def execute_cb(self, goal):
        # print(goal)
        if (goal.scene_id == 0):
            self.scene_func(0, try0_param)
        if (goal.scene_id == 1):
            self.scene_func(1, try1_param)
        if (goal.scene_id == 2):
            self.scene_func(2, try2_param)
        if (goal.scene_id == 3):
            self.scene_func(3, try3_param)
        if (goal.scene_id == 4):
            self.scene_func(4, try4_param)
        if (goal.scene_id == 5):
            self.scene_func(5, try5_param)
        if (goal.scene_id == 6):
            self.scene_func(6, try6_param)
        if (goal.scene_id == 7):
            self.scene_func(7, try7_param)
        if (goal.scene_id == 8):
            self.scene_func(8, try8_param)
        if (goal.scene_id == 9):
            self.sample_scene()

if __name__ == "__main__":
    rospy.init_node('tr_server')
    match_color = phrase_color_from_launch()
    robot_type = phrase_team_from_launch()
    for c in cfg.values():
        c.setFieldColor(match_color)
    fulltask_server = FulltaskSceneHandler()
    rospy.spin()
