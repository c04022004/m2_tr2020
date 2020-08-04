#!/usr/bin/env python
import rospy, actionlib, threading
from m2_move_base.msg import *
from m2_wireless_comm.srv import Request
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import GoalStatus
from m2_tr2020.msg import *

import time
import numpy as np
from numpy import pi
from configs.pursuitConfig import *
from configs.fieldConfig import *
from configs.commConfig import *

match_color = None
robot_type = None

class FulltaskSceneHandler(object):

    def __init__(self):
        self.stop_srv = rospy.Service('stop_srv', Trigger, self.stop_cb)
        self.move_base_client = actionlib.SimpleActionClient('Switch', SwitchModeAction)
        self.move_base_client.wait_for_server()

        self.dji_client = actionlib.SimpleActionClient('dji_try_server', TryAction)
        if robot_type == ROBOT_TR2:
            self.dji_client.wait_for_server()

        self.io_pub_latch = rospy.Publisher('io_board1/io_7/set_state', Bool, queue_size=1) # try latch release/retract (io_7)
        self.io_pub_slider = rospy.Publisher('io_board1/io_0/set_state', Bool, queue_size=1) # try slider release/retract (io_0)
        self.delayed_lifter_thread = None
        self.delayed_slider_thread = None
        self.delayed_call_pr_thread = None
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

        self.command_pr_srv = rospy.ServiceProxy('request_partner_info', Request)

    def stop_cb(self, req):
        self.path_finish_event.set() # set the finish event manually, usual for preempting a goal action
        # you can even set self.move_base_client to an invalid object such that it is uncallable in the remainder of that scene
        # The reverse action should be evemt.clear()
        return (True, "event set")
    
    def gen_intermediate_func(self, x_min, x_max, y_min, y_max, eta, thres):
        def intermediate_func(msg):
            pose = msg.cur_odom.pose.pose.position
            # Breaking out due to pose
            if x_min!=None and x_max!= None and y_min!=None and y_max!=None:
                if pose.x >= x_min and pose.x <= x_max and pose.y >= y_min and pose.y <= y_max:
                    self.path_finish_event.set()
            # elif x_min!=None and x_max!= None:
            #     if pose.x >= x_min and pose.x <= x_max:
            #         self.path_finish_event.set()
            # elif y_min!=None and y_max!= None:
            #     if pose.y >= y_min and pose.y <= y_max:
            #         self.path_finish_event.set()
            # Breaking out due to eta remaining
            if eta!=None:
                twist_lin = msg.cur_odom.twist.twist.linear
                twist_mag = np.hypot(twist_lin.x,twist_lin.y)
                prev_twist_lin = msg.prev_odom.twist.twist.linear
                prev_twist_mag = np.hypot(prev_twist_lin.x,prev_twist_lin.y)
                dt = msg.cur_odom.header.stamp.to_sec()-msg.prev_odom.header.stamp.to_sec()
                acc = np.nan_to_num((twist_mag-prev_twist_mag)/dt)
                dist = np.hypot(eta['dest_x']-pose.x,eta['dest_y']-pose.y)
                prev_pose = msg.prev_odom.pose.pose.position
                prev_dist = np.hypot(eta['dest_x']-prev_pose.x,eta['dest_y']-prev_pose.y)
                # coeff = [-dist, 0.5*acc, twist_mag] # s = ut + 0.5at**2
                # roots = np.roots(coeff)
                # t = roots[roots>=0].min()
                
                # do simulation, using only PD control
                _dist = dist
                _prev_dist = dist
                for i in range(int(eta['time']*100)):
                    cmd_vel = _dist*eta['kP'] + (_dist-_prev_dist)*eta['kD']
                    cmd_vel = min(cmd_vel,eta['vel'])
                    _prev_dist = _dist
                    _dist -= cmd_vel*0.01
                    # rospy.loginfo("dist: %.4f"%_dist)
                    if _dist < eta['dz']:
                        # rospy.loginfo("event trigger when eta=%.4f"%(i/100.0))
                        self.path_finish_event.set()
                        break
            # Breaking out due to eta progress
            if (msg.progress > thres):
                self.path_finish_event.set() # python threading event
        return intermediate_func

    def rospy_shutdown_cb(self):
        rospy.logwarn("shutdown request received")
        if self._as.is_active():
            self.move_base_client.cancel_all_goals()
            if robot_type == ROBOT_TR2:
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
            if robot_type == ROBOT_TR2:
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

    def hook0(self):
        self.ball_guard()

    def hook1(self):
        if robot_type == ROBOT_TR2:
            self.dji_client.send_goal(TryGoal(scene_id=2)) # Pre-lift the rugby
            if self.delayed_slider_thread != None and self.delayed_slider_thread.isAlive():
                self.delayed_slider_thread.cancel()
            self.delayed_slider_thread = threading.Timer(0.5, self.unlock_ball)
            self.delayed_slider_thread.start()

    def hook2(self):
        pass

    def hook3(self):
        pass

    def comm_pr_try_done(self,pr_arg):
        if self.delayed_call_pr_thread != None and self.delayed_call_pr_thread.isAlive():
            self.delayed_call_pr_thread.cancel()
        self.delayed_call_pr_thread = threading.Timer(0.2, self.call_pr, args=(pr_arg,))
        self.delayed_call_pr_thread.start()

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
            self.try_event.wait(0.4)

            self.dji_client.send_goal(TryGoal(scene_id=4))
            self.as_state_decode(self.dji_client.get_state(),"dji_try_server")
            # self.try_event.clear()
            # self.try_event.wait(0.7)

            if self.delayed_slider_thread != None and self.delayed_slider_thread.isAlive():
                self.delayed_slider_thread.cancel()
            self.delayed_slider_thread = threading.Timer(2.0, self.ball_guard)
            self.delayed_slider_thread.start()

    def unlock_ball(self):
        if robot_type == ROBOT_TR2:
            self.io_pub_slider.publish(1) # up

    def ball_guard(self):
        if robot_type == ROBOT_TR1:
            self.io_pub_latch.publish(0) # retract
        elif robot_type == ROBOT_TR2:
            self.io_pub_slider.publish(0) # down

    def call_pr(self, command):
        rospy.loginfo("CALL PR: {}".format(command))
        if robot_type in [ROBOT_TR1,ROBOT_TR2]:
            self.command_pr_srv(command)

    def process_hooks(self, hook_list):
        if hook_list == None:
            return
        for hook_dict in hook_list:
            for hook, arg in hook_dict.items():
                rospy.logdebug("Processing hook self.%s"%hook)
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
