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

MAX_SPEED = 5.0
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
    
    def hook0(self):
        self.ball_guard()
        if robot_type == ROBOT_TR2:
            self.dji_client.send_goal(TryGoal(scene_id=2)) # Pre-lift the rugby

    def hook1(self):
        self.ball_guard()
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
            state = self.dji_client.get_state()
            if state != 3: # not SUCCEEDED
                self.as_state_decode(state,"dji_try_server")

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
            self.dji_client.send_goal(TryGoal(scene_id=3))
            self.dji_client.wait_for_result()
            if self.delayed_slider_thread != None and self.delayed_slider_thread.isAlive():
                self.delayed_slider_thread.cancel()
            self.delayed_slider_thread = threading.Timer(1.0, self.ball_guard)
            self.delayed_slider_thread.start()
            states = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"]
            state = self.dji_client.get_state()
            rospy.logwarn("move_base_client goal done: State=%d %s"%(state, states[state]))

    def latch_io(self, output=0):
        self.io_pub_latch.publish(output)

    def slider_io(self, output=0):
        self.io_pub_slider.publish(output)

    def ball_guard(self):
        if robot_type == ROBOT_TR1:
            self.latch_io(0) # retract
        elif robot_type == ROBOT_TR2:
            self.slider_io(0) # down

    def scene_0(self):
        rospy.loginfo("Fulltask scene 0 start!")
        start_time = time.time()

        # Try sequence - stage1
        self.hook_1()
        self.path_finish_event.clear()
        self.move_base_client.send_goal(
            cfg["scene0_f"].goalConstructor(speed=MAX_SPEED*1.0, radius=2.0, stop_min_speed=0.75,
            velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(), done_cb=self.move_base_client_done_cb)
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        # Try sequence - stage2
        self.hook2()
        self.path_finish_event.clear()
        self.move_base_client.send_goal(
            cfg["scene0_fs"].goalConstructor(speed=MAX_SPEED*0.5, kP=3.0, kI=0.0001, kD=1.0),
            feedback_cb=self.gen_intermediate_func(), done_cb=self.move_base_client_done_cb)
        rospy.loginfo("breaking stage before Try Spot 1")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        # Try sequence - stage6
        self.hook6()
        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def process_hooks(self, hook_list):
        for hook_dict in hook_list:
            for hook, h_arg in hook_dict.items():
                rospy.logdebug("Processing hook self.%s"%hook)
                try:
                    method = getattr(self, hook)
                    if h_arg != None:
                        method(*h_arg)
                    else:
                        method()
                except AttributeError:
                    rospy.logerr("Class %s does not implement %s"%(self.__class__.__name__,hook))

    def scene_func(self, scene_num, params):
        rospy.loginfo("Fulltask scene %d start!"% scene_num)
        start_time = time.time()

        params = []
        stage_cfg = {'hook_func' : [{'hook0':None}],
                     'cfg_name'  : 'scene2_f',
                     'cfg_param' : {'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_min_speed':0.75,
                                    'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.8},
                     'trig_name' : "try_trig",
                     'log_msg'   : "start running for Try Spot 2",}
        params.append(stage_cfg)
        stage_cfg = {'hook_func' : [{'hook1':None}],
                     'cfg_name'  : 'scene2_fs',
                     'cfg_param' : {'speed':MAX_SPEED*0.5, 'kP':3.0, 'kI':0.0001, 'kD':2.5},
                     'trig_name' : "default_trig",
                     'log_msg'   : "breaking stage before Try Spot 2",}
        params.append(stage_cfg)
        stage_cfg = {'hook_func' : [{'hook3':None},{'do_try':None}],
                     'cfg_name'  : None,
                     'cfg_param' : None,
                     'trig_name' : "default_trig",
                     'log_msg'   : "breaking stage before Try Spot 2",}
        params.append(stage_cfg)
        stage_cfg = {'hook_func' : [{'hook3':None}],
                     'cfg_name'  : 'scene2_b',
                     'cfg_param' : {'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_min_speed':0.75,
                                    'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4},
                     'trig_name' : "rec_trig",
                     'log_msg'   : "back to receiving pos",}
        params.append(stage_cfg)
        stage_cfg = {'hook_func' : [{'hook4':None}],
                     'cfg_name'  : 'scene2_bs',
                     'trig_name' : "default_trig",
                     'cfg_param' : {'speed':MAX_SPEED*0.6, 'kP':3.0, 'kI':0.0001, 'kD':4.0},
                     'log_msg'   : "pid into receiving pos",}
        params.append(stage_cfg)
        stage_cfg = {'hook_func' : [{'hook5':None}],
                     'cfg_name'  : None,
                     'cfg_param' : None,
                     'trig_name' : None,
                     'log_msg'   : None,}
        params.append(stage_cfg)

        # --- Try sequence - stage0~4 ---
        for i in range(5):
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


    def scene_3(self):
        rospy.loginfo("Fulltask scene 3 start!")
        start_time = time.time()

        self.ball_guard()
        self.dji_client.send_goal(TryGoal(scene_id=2)) # Pre-lift the ruby
        self.path_finish_event = threading.Event()
        try_trig = try_cfg.getTriggers(MATCH_RED)
        if(match_color == MATCH_BLUE):
            scene3_f_cfg.setFieldColor(MATCH_BLUE)
            try_trig = try_cfg.getTriggers(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene3_f_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func_pose(self.path_finish_event,*try_trig),
            done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 3")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        self.io_pub_slider.publish(1)
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene3_s_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene3_s_cfg.goalConstructor(speed=MAX_SPEED*0.5, kP=3.0, kI=0.0001, kD=2.5),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("breaking stage before Try Spot 3")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        self.do_try()
 
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene3_b_cfg.setFieldColor(MATCH_BLUE)
            try_trig = try_cfg.getTriggers(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene3_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_min_speed=0.75, velocity_shift_kP=6.0,curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event,0.9), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.ball_guard()
        if self.as_check_preempted(): return

        self.dji_client.send_goal(TryGoal(scene_id=0))
        self.dji_client.wait_for_result()
        states = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"]
        state = self.dji_client.get_state()
        rospy.logwarn("move_base_client goal done: State=%d %s"%(state, states[state]))
        # PID stoping at POINT_C
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            pointC_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            pointC_cfg.goalConstructor(speed=MAX_SPEED*0.6, kP=3.0, kI=0.0001, kD=4.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("moving to POINT_C")
        self.path_finish_event.wait()
        self.ball_guard()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_4(self):
        rospy.loginfo("Fulltask scene 4 start!")
        start_time = time.time()

        self.ball_guard()
        self.path_finish_event = threading.Event()
        try_trig = try_cfg.getTriggers(MATCH_RED)
        if(match_color == MATCH_BLUE):
            scene4_f_cfg.setFieldColor(MATCH_BLUE)
            try_trig = try_cfg.getTriggers(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene4_f_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func_pose(self.path_finish_event,*try_trig),
            done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 4")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return
        
        self.io_pub_slider.publish(1)
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene4_s_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene4_s_cfg.goalConstructor(speed=MAX_SPEED*0.5, kP=3.5, kI=0.0005, kD=2.5),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("breaking stage before Try Spot 4")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        self.do_try()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene4_b_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene4_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event),
            done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.ball_guard()
        if self.as_check_preempted(): return

        # PID stoping at POINT_D
        self.ball_guard()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            pointD_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            pointD_cfg.goalConstructor(speed=MAX_SPEED*0.6, kP=3.0, kI=0.0001, kD=4.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("moving to POINT_D")
        self.path_finish_event.wait()
        self.ball_guard()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_5(self):
        rospy.loginfo("Fulltask scene 5 start!")
        start_time = time.time()

        self.ball_guard()
        self.path_finish_event = threading.Event()
        try_trig = try_cfg.getTriggers(MATCH_RED)
        if(match_color == MATCH_BLUE):
            scene5_f_cfg.setFieldColor(MATCH_BLUE)
            try_trig = try_cfg.getTriggers(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene5_f_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=3.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.75),
            feedback_cb=self.gen_intermediate_func_pose(self.path_finish_event,*try_trig),
            done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 5")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        self.io_pub_slider.publish(1)
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene5_s_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene5_s_cfg.goalConstructor(speed=MAX_SPEED*0.6, kP=3.5, kI=0.0005, kD=2.5),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("breaking stage before Try Spot 3")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return
        
        self.do_try()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene5_b_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene5_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=3.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.5),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.ball_guard()
        if self.as_check_preempted(): return

        # PID stoping at POINT_D
        self.ball_guard()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            pointD_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            pointD_cfg.goalConstructor(speed=MAX_SPEED*0.6, kP=3.0, kI=0.0001, kD=4.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("moving to POINT_D")
        self.path_finish_event.wait()
        self.ball_guard()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_6(self):
        rospy.loginfo("Fulltask scene 6 start!")
        start_time = time.time()

        self.path_finish_event.clear()
        self.ball_guard()
        self.move_base_client.send_goal(
            cfg["scene6_f"].goalConstructor(speed=MAX_SPEED*0.5, radius=1.5, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.loginfo("goal to TRSZ")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_7(self):
        rospy.loginfo("Fulltask scene 7 start!")
        start_time = time.time()
        self.ball_guard()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            pointC_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            pointC_cfg.goalConstructor(speed=MAX_SPEED*0.6, kP=3.0, kI=0.0001, kD=4.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("moving to POINT_C")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        rospy.loginfo("move done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_8(self):
        rospy.loginfo("Fulltask scene 8 start!")
        start_time = time.time()
        self.ball_guard()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            pointD_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            pointD_cfg.goalConstructor(speed=MAX_SPEED*0.6, kP=3.0, kI=0.0001, kD=4.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("moving to POINT_D")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        rospy.loginfo("move done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_9(self):
        rospy.loginfo("move done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())
        pass

    def execute_cb(self, goal):
        # print(goal)
        if (goal.scene_id == 0):
            self.scene_0()
        if (goal.scene_id == 1):
            pass
        if (goal.scene_id == 2):
            self.scene_func(2,None)
        if (goal.scene_id == 3):
            self.scene_3()
        if (goal.scene_id == 4):
            self.scene_4()
        if (goal.scene_id == 5):
            self.scene_5()
        if (goal.scene_id == 6):
            self.scene_6()
        if (goal.scene_id == 7):
            self.scene_7()
        if (goal.scene_id == 8):
            self.scene_8()
        if (goal.scene_id == 9):
            self.scene_9()

if __name__ == "__main__":
    rospy.init_node('tr_server')
    match_color = phrase_color_from_launch()
    robot_type = phrase_team_from_launch()
    fulltask_server = FulltaskSceneHandler()
    rospy.spin()
