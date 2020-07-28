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

MAX_SPEED = 4.0

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
        time.sleep(1)

    def setGoal(self, model, param):
        self.path_finish_event = threading.Event()

        if(match_color == MATCH_BLUE):
            model.knotsFlipX()
            
        self.move_base_client.send_goal(
            model.goalConstructor(**param),
            feedback_cb=self.gen_intermediate_func(self.path_finish_event))
        
        self.path_finish_event.wait()
        self.do_try()

    def scene(self, index, segment, param):
        rospy.loginfo("Fulltask scene {0} start!".format(index))
        start_time = time.time()

        if(segment['pursuit_f_mode']):
          param['pursuit_f_param']['pure_pid'] = segment['pid_f_mode']
          self.setGoal(PurePursuitConfig("cubic", CHK_PTS[index]['f_path']), param['pursuit_f_param'])
        
        if(segment['pid_f_mode']):
          self.setGoal(PurePidConfig(CHK_PTS[index]['f_path'][-1], -pi/2), param['pid_f_param'])
        
        if(segment['pursuit_b_mode']):
          param['pursuit_b_param']['pure_pid'] = segment['pid_b_mode']
          self.setGoal(PurePursuitConfig("cubic", CHK_PTS[index]['b_path']), param['pursuit_b_param'])
        
        if(segment['pid_b_mode']):
          self.setGoal(PurePidConfig(CHK_PTS[index]['b_path'][-1], -pi/2), param['pid_b_param'])

        rospy.loginfo("try done. time=%f"%(time.time()-start_time))
        self._as.set_succeeded(FulltaskResult())

    def execute_cb(self, goal):
        # print(goal)
        if (goal.scene_id == 0):
            self.scene(
              index=0,
              segment={
                'pursuit_f_mode':True, 'pursuit_b_mode':False, 'pid_f_mode':False, 'pid_b_mode':False,
              },
              param={
                'pursuit_f_param':{
                  'speed':MAX_SPEED*1.0, 'radius':2.75, 'stop_kI':0.00001, 'stop_kD':0.14, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4
                },
                'pid_f_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                }
              }
            )
        if (goal.scene_id == 1):
            self.scene(
              index=1,
              segment={
                'pursuit_f_mode':True, 'pursuit_b_mode':True, 'pid_f_mode':False, 'pid_b_mode':False,
              },
              param={
                'pursuit_f_param':{
                  'speed':MAX_SPEED*0.8, 'radius':3.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':1.0
                },
                'pursuit_b_param':{
                  'speed':MAX_SPEED*0.8, 'radius':3.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':1.0
                },
                'pid_f_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
                'pid_b_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
              }
            )
        if (goal.scene_id == 2):
            self.scene(
              index=2,
              segment={
                'pursuit_f_mode':True, 'pursuit_b_mode':True, 'pid_f_mode':False, 'pid_b_mode':False,
              },
              param={
                'pursuit_f_param':{
                  'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4
                },
                'pursuit_b_param':{
                  'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4
                },
                'pid_f_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
                'pid_b_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
              }
            )
        if (goal.scene_id == 3):
            self.scene(
              index=3,
              segment={
                'pursuit_f_mode':True, 'pursuit_b_mode':True, 'pid_f_mode':False, 'pid_b_mode':False,
              },
              param={
                'pursuit_f_param':{
                  'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0,'curvature_penalty_kP':0.4
                },
                'pursuit_b_param':{
                  'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0,'curvature_penalty_kP':0.4
                },
                'pid_f_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
                'pid_b_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
              }
            )
        if (goal.scene_id == 4):
            self.scene(
              index=4,
              segment={
                'pursuit_f_mode':True, 'pursuit_b_mode':True, 'pid_f_mode':False, 'pid_b_mode':False,
              },
              param={
                'pursuit_f_param':{
                  'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4
                },
                'pursuit_b_param':{
                  'speed':MAX_SPEED*0.8, 'radius':2.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4
                },
                'pid_f_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
                'pid_b_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
              }
            )
        if (goal.scene_id == 5):
            self.scene(
              index=5,
              segment={
                'pursuit_f_mode':True, 'pursuit_b_mode':True, 'pid_f_mode':True, 'pid_b_mode':True,
              },
              param={
                'pursuit_f_param':{
                  'speed':MAX_SPEED*0.8, 'radius':3.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':1.0,
                },
                'pursuit_b_param':{
                  'speed':MAX_SPEED*0.8, 'radius':3.0, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':1.0,
                },
                'pid_f_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
                'pid_b_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
              }
            )
        if (goal.scene_id == 6):
            self.scene(
              index=6,
              segment={
                'pursuit_f_mode':True, 'pursuit_b_mode':True, 'pid_f_mode':False, 'pid_b_mode':False,
              },
              param={
                'pursuit_b_param':{
                  'speed':MAX_SPEED*0.5, 'radius':1.5, 'stop_kI':0.00001, 'stop_kD':0.1, 'velocity_shift_kP':6.0, 'curvature_penalty_kP':0.4,
                },
                'pid_b_param':{
                  'speed':MAX_SPEED*1.0, 'kP':3.7, 'kI':0.0, 'kD':1.0
                },
              }
            )
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
