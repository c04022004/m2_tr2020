#!/usr/bin/env python
import rospy, actionlib, threading
from m2_move_base.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from math import pi
from m2_tr2020.msg import *
import time
# import shapely.geometry
# import shapely.geometry.polygon

from configs.pursuitConfig import *

MATCH_NONE = 0
MATCH_RED  = 1
MATCH_BLUE = 2


class FulltaskSceneHandler(object):

    def __init__(self):
        stop_srv = rospy.Service('stop_srv', Trigger, self.stop_cb)
        self.move_base_client = actionlib.SimpleActionClient('Switch', SwitchModeAction)
        self.move_base_client.wait_for_server()

        self.cartop_status_pub = rospy.Publisher("cartop_status", Marker, queue_size = 1)
        self.omni_frame = rospy.get_param("~base_frame", "omni_base")

        self._action_name = "tr_server"
        self._as = actionlib.SimpleActionServer(self._action_name, FulltaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.stop_polygon_pub = rospy.Publisher("stop_polygon", PolygonStamped, queue_size = 0)

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
        time.sleep(0.1)

    def scene_1(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene1_cfg.knotsFlipX()
        self.move_base_client.send_goal(scene1_cfg.goalConstructor(True), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_2(self):
        rospy.loginfo("Fulltask scene 2 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene2_cfg.knotsFlipX()
        self.move_base_client.send_goal(scene2_cfg.goalConstructor(True), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        self.do_try()
        scene2_cfg.reverseKnots()
        self.move_base_client.send_goal(scene2_cfg.goalConstructor(False), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_3(self):
        rospy.loginfo("Fulltask scene 3 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene3_cfg.knotsFlipX()
        self.move_base_client.send_goal(scene3_cfg.goalConstructor(True), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        self.do_try()
        scene3_cfg.reverseKnots()
        self.move_base_client.send_goal(scene3_cfg.goalConstructor(False), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_4(self):
        rospy.loginfo("Fulltask scene 4 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene4_cfg.knotsFlipX()
        self.move_base_client.send_goal(scene4_cfg.goalConstructor(True), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        self.do_try()
        scene4_cfg.reverseKnots()
        self.move_base_client.send_goal(scene4_cfg.goalConstructor(False), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_5(self):
        rospy.loginfo("Fulltask scene 5 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene5_cfg.knotsFlipX()
        self.move_base_client.send_goal(scene5_cfg.goalConstructor(True), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        self.do_try()
        scene5_cfg.reverseKnots()
        self.move_base_client.send_goal(scene5_cfg.goalConstructor(False), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_6(self):
        rospy.loginfo("Fulltask scene 6 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene6_cfg.knotsFlipX()
        self.move_base_client.send_goal(scene6_cfg.goalConstructor(True), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        self.do_try()
        scene6_cfg.reverseKnots()
        self.move_base_client.send_goal(scene6_cfg.goalConstructor(False), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_7(self):
        rospy.loginfo("Fulltask scene 7 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_BLUE):
            scene7_cfg.knotsFlipX()
        self.move_base_client.send_goal(scene7_cfg.goalConstructor(True), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())
        
    def execute_cb(self, goal):
        # print(goal)
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

if __name__ == "__main__":
    rospy.init_node('tr_server')
    color = rospy.get_param("~color", "red")
    if color == "red":
        match_color = MATCH_RED
    else:
        match_color = MATCH_BLUE
    fulltask_server = FulltaskSceneHandler()
    rospy.spin()
