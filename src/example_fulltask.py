#!/usr/bin/env python
import rospy, actionlib, threading
from m2_move_base.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger

def PurePursuitConstructor(knots, target_z = 0.0, vel = 1.8):
    # returns a SwitchModeGoal set to PURE_PURSUIT, with some default values

    pure_pursuit_data = PurePursuitData()
    pure_pursuit_data.label = "bezier"
    pure_pursuit_data.spline_type = pure_pursuit_data.BEZIER
    
    pure_pursuit_data.knots = knots
    pure_pursuit_data.point_density = 10000
    
    pure_pursuit_data.velocity_xy_magnitude = vel

    pure_pursuit_data.velocity_z_kP = 1
    pure_pursuit_data.velocity_z_max = 2
    pure_pursuit_data.target_z = target_z

    pure_pursuit_data.lookahead_distance = 0.2

    pure_pursuit_data.stop_type = pure_pursuit_data.STOP_PID
    pure_pursuit_data.stop_pid_radius = 0.5
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
            pose = msg.cur_pos.pose.pose
            rospy.loginfo_throttle(0.5, "Intermediate position: %f %f %f"%(pose.position.x, pose.position.y, msg.progress))
            if (msg.progress > 0.9):
                event.set() # python threading event
        return intermediate_func

    def scene_1(self):
        rospy.loginfo("Fulltask scene 1 start!")
        self.status_dict["status"] = "go"

        chk_pts = [Point(x=point[0], y=point[1]) for point in
                    [ (0.462927,0.50826),(8.77503,-1.79321),(9.01469,8.66005),(-0.691011,7.55541),(0.217586,0.717667) ]]

        # IMPORTANT: A NEW EVENT OBJECT SHOULD BE CREATED (ASSIGNED) FOR EACH GOAL
        self.path_finish_event = threading.Event() # python threading event.
        # IMPORTANT: A NEW EVENT OBJECT SHOULD BE CREATED (ASSIGNED) FOR EACH GOAL

        self.move_base_client.send_goal(PurePursuitConstructor(knots = chk_pts),
                                        feedback_cb=self.gen_intermediate_func(self.path_finish_event))
                                # feedback_cb will be called whenever there is an update to chassis_odom (100hz),
                                #             while this goal is the latest goal
                                #
                                # useful for handling events that will happen within a path, e.g.
                                #     - stopping/changing a path in the middle (this example)
                                #     - performing car top fsm actions during a path (e.g. do something when the car reaches a certain odom)
                                # however, all callbacks should end quickly (no time sleeps / blocking service calls!)
                                
        rospy.loginfo("spline goal sent")

        self.path_finish_event.wait() # python threading event waits until the event to continue is set by the feedback_cb
        # event wait is a better method than using a while loop, as it saves CPU processing power
        rospy.loginfo("at 90%, nearly done")

        # publish a stop mode. for all modes, goal without feedback_cb is also ok
        self.move_base_client.send_goal(SwitchModeGoal(target_mode=SwitchModeGoal().FIXED_VELOCITY))
        rospy.loginfo("stop goal sent")

        self.status_dict["status"] = "stop"
        rospy.spin()

if __name__ == "__main__":
    # in actual robocon task, action client should be used here. this one is just for simplicity.

    rospy.init_node('example_fulltask')
    fulltask = FulltaskSceneHandler()
    fulltask.scene_1()
