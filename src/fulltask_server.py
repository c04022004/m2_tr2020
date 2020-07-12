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

POINT_S = (0.5, 9.5)
POINT_C = (0.7, 0.9)
POINT_1 = (5.72, 3.09+4.0*0.03+4*1.3)
POINT_2 = (5.72, 3.09+3.0*0.03+3*1.3)
POINT_3 = (5.72, 3.09+2.0*0.03+2*1.3)
POINT_4 = (5.72, 3.09+1.0*0.03+1*1.3)
POINT_5 = (5.72, 3.09+0.5*0.03+0*1.3)

CHK_PTS = [
    [ POINT_S, POINT_C ],                                                             # scene 1
    [ POINT_C,(2.26316,2.88727),(4.47998,4.81193),POINT_1 ],                          # scene 2
    [ POINT_C,(2.61315,2.66303),(4.57492,4.80013),POINT_2 ],                          # scene 3
    [ POINT_C,(2.55856,2.84425),POINT_3 ],                                            # scene 4
    [ POINT_C,(2.3, 3),POINT_4 ],                                                     # scene 5
    [ POINT_C,(1.31412,2.53211),(3.79225,3.88193),POINT_5 ],                          # scene 6
    [ POINT_C, POINT_S ],                                                             # scene 7
]

TEST_SPEED_F = 3.0
TEST_SPEED_B = 3.5
RADIUS_F = 2.0
RADIUS_B = 2.0
CONFIG = [
    {'vel_f':TEST_SPEED_F, 'vel_b':TEST_SPEED_B, 'radius_f': RADIUS_F, 'radius_b': RADIUS_B},
    {'vel_f':TEST_SPEED_F, 'vel_b':TEST_SPEED_B, 'radius_f': RADIUS_F, 'radius_b': RADIUS_B},
    {'vel_f':TEST_SPEED_F, 'vel_b':TEST_SPEED_B, 'radius_f': RADIUS_F, 'radius_b': RADIUS_B},
    {'vel_f':TEST_SPEED_F, 'vel_b':TEST_SPEED_B, 'radius_f': RADIUS_F, 'radius_b': RADIUS_B},
    {'vel_f':TEST_SPEED_F, 'vel_b':TEST_SPEED_B, 'radius_f': RADIUS_F, 'radius_b': RADIUS_B},
    {'vel_f':0.5, 'vel_b':0.5, 'radius_f': RADIUS_F, 'radius_b': RADIUS_B},
    {'vel_f':1, 'vel_b':1, 'radius_f': RADIUS_F, 'radius_b': RADIUS_B},
]


def PurePursuitConstructor(knots, target_z = 0.0, vel = 1.0, radius=0.5):
    # returns a SwitchModeGoal set to PURE_PURSUIT, with some default values

    pure_pursuit_data = PurePursuitData()
    #pure_pursuit_data.label = "bezier"
    pure_pursuit_data.spline_type = pure_pursuit_data.CUBIC
    
    pure_pursuit_data.knots = knots
    pure_pursuit_data.point_density = 10000
    
    pure_pursuit_data.velocity_xy_magnitude = vel

    pure_pursuit_data.velocity_z_kP = 3
    pure_pursuit_data.velocity_z_max = 2
    pure_pursuit_data.target_z = target_z

    pure_pursuit_data.lookahead_distance = 0.0
    pure_pursuit_data.vector_policy = pure_pursuit_data.TANGENTIAL_PID_TWIST
    pure_pursuit_data.velocity_shift_kP = 4.0

    pure_pursuit_data.stop_type = pure_pursuit_data.STOP_PID
    pure_pursuit_data.stop_pid_radius = radius
    pure_pursuit_data.stop_kP = 1.5

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
            # rospy.loginfo_throttle(0.5, "Intermediate position: %f %f %f"%(pose.position.x, pose.position.y, msgx.progress))
            if (msg.progress > 0.95):
                event.set() # python threading event
        return intermediate_func

    def do_try(self):
        time.sleep(10)
        pass

    def scene_1(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_RED):
            chk_pts = [Point(x=point[0], y=point[1]) for point in CHK_PTS[0]]
            t_z = -pi/2
        else:
            chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in CHK_PTS[0]]
            t_z = pi/2
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[0]['vel_f'], radius=CONFIG[1]['radius_f']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_2(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_RED):
            chk_pts = [Point(x=point[0], y=point[1]) for point in CHK_PTS[1]]
            t_z = -pi/2
        else:
            chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in CHK_PTS[1]]
            t_z = pi/2
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[1]['vel_f'], radius=CONFIG[2]['radius_f']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        time.sleep(2)
        chk_pts.reverse()
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[1]['vel_b'], radius=CONFIG[2]['radius_b']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_3(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_RED):
            chk_pts = [Point(x=point[0], y=point[1]) for point in CHK_PTS[2]]
            t_z = -pi/2
        else:
            chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in CHK_PTS[2]]
            t_z = pi/2
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[2]['vel_f'], radius=CONFIG[3]['radius_f']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        time.sleep(2)
        chk_pts.reverse()
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[2]['vel_b'], radius=CONFIG[3]['radius_b']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_4(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_RED):
            chk_pts = [Point(x=point[0], y=point[1]) for point in CHK_PTS[3]]
            t_z = -pi/2
        else:
            chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in CHK_PTS[3]]
            t_z = pi/2
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[3]['vel_f'], radius=CONFIG[4]['radius_f']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        time.sleep(2)
        chk_pts.reverse()
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[3]['vel_b'], radius=CONFIG[4]['radius_b']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_5(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_RED):
            chk_pts = [Point(x=point[0], y=point[1]) for point in CHK_PTS[4]]
            t_z = -pi/2
        else:
            chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in CHK_PTS[4]]
            t_z = pi/2
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[4]['vel_f'], radius=CONFIG[5]['radius_f']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        time.sleep(2)
        chk_pts.reverse()
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[4]['vel_b'], radius=CONFIG[5]['radius_b']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_6(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_RED):
            chk_pts = [Point(x=point[0], y=point[1]) for point in CHK_PTS[5]]
            t_z = -pi/2
        else:
            chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in CHK_PTS[5]]
            t_z = pi/2
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[5]['vel_f'], radius=CONFIG[6]['radius_f']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()
        
        time.sleep(2)

        chk_pts.reverse()
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[5]['vel_b'], radius=CONFIG[6]['radius_b']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        rospy.loginfo("goal to receive pos")
        self.path_finish_event.wait()

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def scene_7(self):
        rospy.loginfo("Fulltask scene 1 start!")
        start_time = time.time()

        self.path_finish_event = threading.Event()
        if(match_color == MATCH_RED):
            chk_pts = [Point(x=point[0], y=point[1]) for point in CHK_PTS[6]]
            t_z = -pi/2
        else:
            chk_pts = [Point(x=(13.3 - point[0]), y=point[1]) for point in CHK_PTS[6]]
            t_z = pi/2
        self.move_base_client.send_goal(PurePursuitConstructor(knots=chk_pts, target_z=t_z, vel=CONFIG[6]['vel_b'], radius=CONFIG[6]['radius_b']), feedback_cb=self.gen_intermediate_func(self.path_finish_event))
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
