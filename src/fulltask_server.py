#!/usr/bin/env python
import rospy, actionlib, threading
from m2_move_base.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from math import pi
from m2_tr2020.msg import *
import time
from actionlib_msgs.msg import GoalStatus
# import shapely.geometry
# import shapely.geometry.polygon

from configs.pursuitConfig import *


MATCH_NONE = 0
MATCH_RED  = 1
MATCH_BLUE = 2

MAX_SPEED = 2.0

class FulltaskSceneHandler(object):

    def __init__(self):
        stop_srv = rospy.Service('stop_srv', Trigger, self.stop_cb)
        self.move_base_client = actionlib.SimpleActionClient('Switch', SwitchModeAction)
        self.move_base_client.wait_for_server()

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

    def gen_intermediate_func(self, event):
        def intermediate_func(msg):
            # pose = msg.cur_pos.pose.pose
            # rospy.loginfo_throttle(0.5, "Intermediate position: %f %f %f"%(pose.position.x, pose.position.y, msg.progress))
            if (msg.progress > 0.99):
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

    def do_try(self):
        time.sleep(0.5)

    def scene_0(self):
        rospy.loginfo("Fulltask scene 0 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene0_f_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene0_f_cfg.goalConstructor(speed=MAX_SPEED*1.0, radius=2.75, stop_kI=0.00001, stop_kD=0.14, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_1(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene1_f_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene1_f_cfg.goalConstructor(speed=MAX_SPEED*0.5, radius=2.0, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=1.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 1")
        self.path_finish_event.wait()
        
        self.do_try()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene1_b_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene1_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=3.0, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=1.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_2(self):
        rospy.loginfo("Fulltask scene 2 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene2_f_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene2_f_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=0.8),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 2")
        self.path_finish_event.wait()
        
        if not self._as.is_active() :
            rospy.logwarn("shutdown request received, or fulltask_server was preempted. returning...")
            rospy.logwarn("_as.is_active(): %d"%(self._as.is_active()))
            rospy.logwarn("_as.is_preempt_requested: %d"%(self._as.is_preempt_requested()))
            return

        self.do_try()
        
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene2_b_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene2_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=0.8),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event), done_cb=self.gen_move_base_client_done_cb(self.path_finish_event))
        rospy.on_shutdown(self.gen_shutdown_func(self.path_finish_event))
        self._as.register_preempt_callback(self.gen_preempt_cb(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()

        if not self._as.is_active():
            rospy.logwarn("shutdown request received, or fulltask_server was preempted. returning...")
            rospy.logwarn("_as.is_active(): %d"%(self._as.is_active()))
            rospy.logwarn("_as.is_preempt_requested: %d"%(self._as.is_preempt_requested()))
            return

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())


    def scene_3(self):
        rospy.loginfo("Fulltask scene 3 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene3_f_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene3_f_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_kI=0.00001, stop_kD=2.0, velocity_shift_kP=6.0, curvature_penalty_kP=0.8),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 3")
        self.path_finish_event.wait()
        
        self.do_try()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene3_b_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene3_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_kI=0.00001, stop_kD=2.0, velocity_shift_kP=6.0,curvature_penalty_kP=0.8),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_4(self):
        rospy.loginfo("Fulltask scene 4 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene4_f_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene4_f_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 4")
        self.path_finish_event.wait()
        
        self.do_try()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene4_b_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene4_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=2.0, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_5(self):
        rospy.loginfo("Fulltask scene 5 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene5_f_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene5_f_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=3.0, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=1.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to Try Spot 5")
        self.path_finish_event.wait()
        
        self.do_try()
        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene5_b_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene5_b_cfg.goalConstructor(speed=MAX_SPEED*0.8, radius=3.0, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=1.0),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receiving pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_6(self):
        rospy.loginfo("Fulltask scene 6 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene0_b_cfg.knotsFlipX()
        self.move_base_client.send_goal(
            scene0_b_cfg.goalConstructor(speed=MAX_SPEED*0.5, radius=1.5, stop_kI=0.00001, stop_kD=0.2, velocity_shift_kP=6.0, curvature_penalty_kP=0.4),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to TRSZ")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())
        
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
        # if (goal.scene_id == 7):
        #     self.scene_7()

if __name__ == "__main__":
    rospy.init_node('tr_server')
    color = rospy.get_param("~color", "red")
    if color == "red":
        match_color = MATCH_RED
    else:
        match_color = MATCH_BLUE
    fulltask_server = FulltaskSceneHandler()
    rospy.spin()
