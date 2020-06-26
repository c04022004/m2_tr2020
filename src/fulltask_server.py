#!/usr/bin/env python
import rospy, actionlib, threading
from m2_move_base.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from math import pi
from m2_tr2020.msg import *
import time

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

    pure_pursuit_data.stop_type = pure_pursuit_data.STOP_PID
    pure_pursuit_data.stop_pid_radius = radius
    pure_pursuit_data.stop_kP = 2.0

    goal = SwitchModeGoal(target_mode=SwitchModeGoal().PURE_PURSUIT, pure_pursuit_data=pure_pursuit_data)
    return goal

class FulltaskSceneHandler(object):

    def __init__(self):
        stop_srv = rospy.Service('stop_srv', Trigger, self.stop_cb)
        self.move_base_client = actionlib.SimpleActionClient('Switch', SwitchModeAction)
        self.move_base_client.wait_for_server()

        self.cartop_status_pub = rospy.Publisher("cartop_status", Marker, queue_size = 1)
        self.omni_frame = rospy.get_param("~base_frame", "omni_base")
        rospy.Timer(rospy.Duration(0.05), self.publish_cartop_status)
        self.status_dict = {}

        self._action_name = "tr_server"
        self._as = actionlib.SimpleActionServer(self._action_name, FulltaskAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def publish_cartop_status(self, args):
        marker = Marker()
        marker.header.frame_id = self.omni_frame
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.ns = "cartop_status"
        marker.action = Marker.ADD
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.pose.position.z = 0.5

        for key in self.status_dict:
            marker.text += key + ": " + self.status_dict[key] + "\n"

        self.cartop_status_pub.publish(marker)

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
        if (goal.scene_id == 2):
            self.scene_2()

    def scene_1(self):
        # IMPORTANT: A NEW EVENT OBJECT SHOULD BE CREATED (ASSIGNED) FOR EACH GOAL
        self.path_finish_event = threading.Event() # python threading event.
        # IMPORTANT: A NEW EVENT OBJECT SHOULD BE CREATED (ASSIGNED) FOR EACH GOAL

        rospy.loginfo("Fulltask scene 1 start!")

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (0.5, 9.5), (0.5, 1.5) ]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots = chk_pts, target_z=-pi/2), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

    def scene_2(self):
        rospy.loginfo("Fulltask try 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (0.5,1.5),(4.03409,3.80815),(5.98142,3.14402) ]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots = chk_pts, target_z=-pi/2), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to tryspot 1")
        self.path_finish_event.wait()

        rospy.loginfo("placing ball")
        rospy.sleep(0.5)
        rospy.loginfo("done try")

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (5.98142,3.14402), (4.03409,3.80815), (0.5,1.5)]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots = chk_pts, target_z=-pi/2), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    
    def scene_3(self):
        rospy.loginfo("Fulltask try 2 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (1.0,1.5),(3.27953,3.77892),(5.65563,4.47238) ]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots = chk_pts, target_z=-pi/2), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to tryspot 2")
        self.path_finish_event.wait()

        rospy.loginfo("placing ball")
        rospy.sleep(1.5)
        rospy.loginfo("done try")

        start_time = time.time()

        self.path_finish_event = threading.Event()
        chk_pts = [Point(x=point[0], y=point[1]) for point in [ (5.65563,4.47238), (3.27953,3.77892), (1.0,1.5)]]
        self.move_base_client.send_goal(PurePursuitConstructor(knots = chk_pts, target_z=-pi/2, vel=4.0, radius=1.0), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. one way time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())
    


    def execute_cb(self, goal):
        # print(goal)
        if (goal.scene_id == 1):
            self.scene_1()
        if (goal.scene_id == 2):
            self.scene_2()
        if (goal.scene_id == 3):
            self.scene_3()

if __name__ == "__main__":
    rospy.init_node('example_fulltask')
    fulltask_server = FulltaskSceneHandler()
    rospy.spin()
