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
# import shapely.geometry
# import shapely.geometry.polygon

from configs.pursuitConfig import *


# MATCH_NONE = 0
# MATCH_RED  = 1
# MATCH_BLUE = 2

MAX_SPEED = 5.0

class FulltaskSceneHandler(object):

    def __init__(self):
        self.stop_srv = rospy.Service('stop_srv', Trigger, self.stop_cb)
        self.move_base_client = actionlib.SimpleActionClient('Switch', SwitchModeAction)
        self.move_base_client.wait_for_server()

        self.io_pub = rospy.Publisher('io_board1/io_7/set_state', Bool, queue_size=1) # try latch release/retract (io_7)
        self.delayed_lifter_thread = None

        self.cartop_status_pub = rospy.Publisher("cartop_status", Marker, queue_size = 1)
        self.omni_frame = rospy.get_param("~base_frame", "omni_base")

        self._action_name = "tr_server"
        self._as = actionlib.SimpleActionServer(self._action_name, FulltaskAction, execute_cb=self.execute_cb, auto_start=False)
        self.stop_polygon_pub = rospy.Publisher("stop_polygon", PolygonStamped, queue_size = 0)

        self._as.start()

        if (match_color == MATCH_BLUE):
            print("tr_server started with BLUE")
        if (match_color == MATCH_RED):
            print("tr_server started with RED")

    def stop_cb(self, req):
        self.path_finish_event.set() # set the finish event manually, usual for preempting a goal action
        # you can even set self.move_base_client to an invalid object such that it is uncallable in the remainder of that scene

        return (True, "event set")

    def gen_intermediate_func(self, event, thres=0.98):
        def intermediate_func(msg):
            # pose = msg.cur_pos.pose.pose
            # rospy.loginfo_throttle(0.5, "Intermediate position: %f %f %f"%(pose.position.x, pose.position.y, msg.progress))
            if (msg.progress > thres):
                event.set() # python threading event
            
        return intermediate_func
    
    def gen_intermediate_func_pose(self, event, x_min, x_max, y_min, y_max, thres=0.99):
        def intermediate_func(msg):
            pose = msg.cur_pos.pose.pose
            pose.position.x, pose.position.y
            if pose.position.x >= x_min and pose.position.x <= x_max and\
                pose.position.y >= y_min and pose.position.y <= y_max:
                event.set()
            
            if (msg.progress > thres):
                event.set() # python threading event
            
        return intermediate_func

    def gen_shutdown_func(self, event):
        def shutdown_func():
            rospy.logwarn("shutdown request received")
            if self._as.is_active():
                self.move_base_client.cancel_all_goals()
                self._as.set_preempted()
            event.set()
        return shutdown_func

    def gen_move_base_client_done_cb(self, event):
        def done_cb(state, result):
            # from actionlib_msgs.msg import GoalStatus
            # State enum: http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
            states = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"]
            rospy.logwarn("move_base_client goal done: State=%d %s"%(state, states[state]))
            event.set()
        return done_cb

    def gen_preempt_cb(self, event):
        def preempt_cb(): # only called when the goal is cancelled externally outisde of this node (e.g. publishing to tr_server/cancel)
            rospy.logwarn("fulltask goal preempt request")
            if self._as.is_active():
                self.move_base_client.cancel_all_goals()
                self._as.set_preempted()
            event.set()
        return preempt_cb

    def gen_try_intermediate_func(self, event, try_y):
        # try_polygon_points = [
        #     (5.65, try_y-0.05),
        #     (5.65, try_y+0.05),
        #     # (6.65, 3.09+0.05),
        #     # (6.65, 3.09-0.05)
        #     (7.65, try_y+0.05),
        #     (7.65, try_y-0.05)
        # ]
        # shapely_polygon = shapely.geometry.Polygon(try_polygon_points)
        # def intermediate_func(msg):
        #     stop_polygon_msg = PolygonStamped()
        #     stop_polygon_msg.header.frame_id = "map"

        #     for pair in try_polygon_points:
        #         x = pair[0]
        #         y = pair[1]

        #         p = Point()
        #         p.y = y
        #         p.z = 0.02
        #         stop_polygon_msg.polygon.points.append(p)
            
        #     self.stop_polygon_pub.publish(stop_polygon_msg)
        #     pose = msg.cur_pos.pose.pose
        #     # rospy.loginfo_throttle(0.5, "Intermediate position: %f %f %f"%(pose.position.x, pose.position.y, msg.progress))
        #     shapely_point = shapely.geometry.Point(pose.position.x, pose.position.y)

        #     if (shapely_polygon.contains(shapely_point)):
        #         stop_polygon_msg = PolygonStamped()
        #         stop_polygon_msg.header.frame_id = "map"
        #         self.stop_polygon_pub.publish(stop_polygon_msg)
        #         event.set() # python threading event
                
        return intermediate_func

    def as_check_preempted(self):
        if not self._as.is_active() :
            rospy.logwarn("shutdown request received, or fulltask_server was preempted. returning...")
            rospy.logwarn("_as.is_active(): %d"%(self._as.is_active()))
            rospy.logwarn("_as.is_preempt_requested: %d"%(self._as.is_preempt_requested()))
            return True
        return False

    def do_try(self):
        self.io_pub.publish(1)
        time.sleep(1.0)
        self.latch_lock()
        # if self.delayed_lifter_thread != None and self.delayed_lifter_thread.isAlive():
        #     self.delayed_lifter_thread.cancel()
        # self.delayed_lifter_thread = threading.Timer(0.1, self.latch_lock)
        # self.delayed_lifter_thread.start()
    
    def latch_io(self, output=0):
        self.io_pub.publish(output)

    def latch_lock(self):
        self.latch_io(0) # retract

    def scene_0(self):
        rospy.loginfo("Fulltask scene 0 start!")
        start_time = time.time()

        self.latch_lock()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene0_f_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene0_f_cfg.goalConstructor(speed=MAX_SPEED*1.0, radius=2.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.latch_lock()
        if self.as_check_preempted(): return

        self.latch_lock()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene0_s_cfg.positionFlipX()
        self.move_base_client.send_goal(
            scene0_s_cfg.goalConstructor(speed=MAX_SPEED*0.5, kP=3.0, kI=0.0001, kD=1.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("breaking stage before Try Spot 1")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_1(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.latch_lock()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene1_f_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene1_f_cfg.goalConstructor(speed=MAX_SPEED*0.5, radius=2.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 1")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        self.latch_lock()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene1_s_cfg.positionFlipX()
        self.move_base_client.send_goal(
            scene1_s_cfg.goalConstructor(speed=MAX_SPEED*0.5, kP=2.0, kI=0.0001, kD=2.5),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("breaking stage before Try Spot 1")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        self.do_try()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene1_b_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene1_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=3.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.latch_lock()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_2(self):
        rospy.loginfo("Fulltask scene 2 start!")
        start_time = time.time()

        self.latch_lock()
        self.path_finish_event = threading.Event()
        try_trig = try_cfg.getTriggers(MATCH_RED)
        if(match_color == MATCH_BLUE):
            scene2_f_cfg.knotsFlipX()
            try_trig = try_cfg.getTriggers(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene2_f_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.8),
            feedback_cb=self.gen_intermediate_func_pose(self.path_finish_event,*try_trig),
            done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 2")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        self.latch_lock()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene2_s_cfg.positionFlipX()
        self.move_base_client.send_goal(
            scene2_s_cfg.goalConstructor(speed=MAX_SPEED*0.5, kP=3.0, kI=0.0001, kD=2.5),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("breaking stage before Try Spot 2")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        self.do_try()
        
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene2_b_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene2_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.latch_lock()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())


    def scene_3(self):
        rospy.loginfo("Fulltask scene 3 start!")
        start_time = time.time()

        self.latch_lock()
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

        self.latch_lock()
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
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.latch_lock()
        if self.as_check_preempted(): return
    
        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_4(self):
        rospy.loginfo("Fulltask scene 4 start!")
        start_time = time.time()

        self.latch_lock()
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
            feedback_cb=self.gen_intermediate_func_pose(self.path_finish_event,*try_trig),
            done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.latch_lock()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_5(self):
        rospy.loginfo("Fulltask scene 5 start!")
        start_time = time.time()

        self.latch_lock()
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

        self.latch_lock()
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
            scene5_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=3.0, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.75),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()
        self.latch_lock()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_6(self):
        rospy.loginfo("Fulltask scene 6 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene0_b_cfg.setFieldColor(MATCH_BLUE)

        self.latch_lock()
        self.move_base_client.send_goal(
            scene0_b_cfg.goalConstructor(speed=MAX_SPEED*0.5, radius=1.5, stop_min_speed=0.75, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to TRSZ")
        self.path_finish_event.wait()
        if self.as_check_preempted(): return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_7(self):
        rospy.loginfo("Fulltask scene 7 start!")
        start_time = time.time()
        self.latch_lock()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene7_s_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene7_s_cfg.goalConstructor(speed=MAX_SPEED*0.6, kP=3.0, kI=0.0001, kD=4.0),
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
        self.latch_lock()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene8_s_cfg.setFieldColor(MATCH_BLUE)
        self.move_base_client.send_goal(
            scene8_s_cfg.goalConstructor(speed=MAX_SPEED*0.6, kP=3.0, kI=0.0001, kD=4.0),
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
            self.scene_1()
        if (goal.scene_id == 2):
            self.scene_2()
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
    color = rospy.get_param("~color", "red")
    if color == "red":
        match_color = MATCH_RED
    elif color == "blue":
        match_color = MATCH_BLUE
    else:
        match_color = MATCH_RED
        rospy.logwarn("No valid color specified, defaulting to RED")
    fulltask_server = FulltaskSceneHandler()
    rospy.spin()
