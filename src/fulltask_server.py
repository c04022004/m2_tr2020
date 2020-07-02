#!/usr/bin/env python
import rospy, actionlib, threading
from m2_move_base.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from math import pi
from m2_tr2020.msg import *
import time

MATCH_NONE = 0
MATCH_RED  = 1
MATCH_BLUE = 2

def PurePursuitConstructor(knots, target_z = 0.0, vel = 1.0, radius=0.5):
    # returns a SwitchModeGoal set to PURE_PURSUIT, with some default values

    pure_pursuit_data = PurePursuitData()
    pure_pursuit_data.label = "bezier"
    pure_pursuit_data.spline_type = pure_pursuit_data.CUBIC
    
    pure_pursuit_data.knots = knots
    pure_pursuit_data.point_density = 10000
    
    pure_pursuit_data.velocity_xy_magnitude = vel

    pure_pursuit_data.velocity_z_kP = 1
    pure_pursuit_data.velocity_z_max = 2
    pure_pursuit_data.target_z = target_z

    pure_pursuit_data.lookahead_distance = 0.2
    pure_pursuit_data.vector_policy = pure_pursuit_data.TANGENTIAL_PID_TWIST

    pure_pursuit_data.stop_type = pure_pursuit_data.STOP_PID
    pure_pursuit_data.stop_pid_radius = radius
    pure_pursuit_data.stop_kP = 1.0

    goal = SwitchModeGoal(target_mode=SwitchModeGoal().PURE_PURSUIT, pure_pursuit_data=pure_pursuit_data)
    return goal

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
            if (msg.progress > 0.95):
                event.set() # python threading event
        return intermediate_func

    def scene_1(self):
        # IMPORTANT: A NEW EVENT OBJECT SHOULD BE CREATED (ASSIGNED) FOR EACH GOAL
        self.path_finish_event = threading.Event() # python threading event.
        # IMPORTANT: A NEW EVENT OBJECT SHOULD BE CREATED (ASSIGNED) FOR EACH GOAL
        
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (0.75, 9.5), (0.75, 1.5) ]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=-pi/2, vel=3.0, radius=1.5), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_2(self):
        rospy.loginfo("Fulltask try 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (0.5,1.5),(4.03409,3.80815),(5.98142,3.14402) ]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=-pi/2), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to tryspot 1")
        self.path_finish_event.wait()

        rospy.loginfo("placing ball")
        rospy.sleep(0.5)
        rospy.loginfo("done try")

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (5.98142,3.14402), (4.03409,3.80815), (0.5,1.5)]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=-pi/2), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    
    def scene_3(self):
        rospy.loginfo("Fulltask try 2 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (1.0,1.5),(3.27953,3.77892),(5.65563,4.47238) ]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=-pi/2), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to tryspot 2")
        self.path_finish_event.wait()

        rospy.loginfo("placing ball")
        rospy.sleep(1.5)
        rospy.loginfo("done try")

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (5.65563,4.47238), (3.27953,3.77892), (1.0,1.5)]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=0.0, vel=4.0, radius=1.0), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. one way time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())
    
    def scene_4(self):
        rospy.loginfo("Fulltask scene 4 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (0.75, 1.5), (0.75, 9.5) ]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=-pi/2, vel=3.0, radius=1.5), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
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

if __name__ == "__main__":
    rospy.init_node('tr_server')
    match_color = rospy.get_param("~match_color")
    fulltask_server = FulltaskSceneHandler()
    rospy.spin()
