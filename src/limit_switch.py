#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool,Float32


#io_topics = ["/io_"+str(i)+"/get_state" for i in range(8)]
MATCH_NONE = 0
MATCH_RED  = 1
MATCH_BLUE = 2

initial_value = {"init_x": rospy.get_param("/limit_switch_reset/reset_x"), 
                "init_y": rospy.get_param("/limit_switch_reset/reset_y"),
                "init_z": rospy.get_param("/limit_switch_reset/reset_z")}


class IOState(object):
    def __init__(self,id=0):
        self.id = id
        self.state = False
        self.counter = 0
        self.subscriber = None

    def state_cb(self,msg):
        self.state = msg.data
        #if self.state:
        #    if self.counter
        #else:
        #    self.state = msg.data

    def get_state(self):
        return self.state

    def set_subscriber(self):
        try:
            topic = "/io_"+str(self.id)+"/get_state"
            
            self.subscriber = rospy.Subscriber(topic, Bool, self.state_cb)
            return True
        except Exception as ex:
            rospy.loginfo(ex)
            return False


limit_switches = [ IOState(id=i) for i in range(8)]
subscribers =[]
for i in range(8):
    subscribers.append(rospy.Subscriber("/io_board1/io_"+str(i)+"/get_state", Bool, limit_switches[i].state_cb))

odom_setx_pub = rospy.Publisher("/odom_set_x", Float32)
odom_sety_pub = rospy.Publisher("/odom_set_y", Float32)
odom_setz_pub = rospy.Publisher("/odom_set_z", Float32)
# match_color = MATCH_RED

if __name__ == "__main__":
    rospy.init_node('limit_switch_reset')
    rate = rospy.Rate(100)

    # Robot construction
    # 
    #        m1 / ----- \ m0        y
    #           |       |           ^
    #  sw(2,3)  |       | sw(0,1)   |--> x
    #           |       | 
    #        m2 \ ----- / m3
    #            sw(4,5)

    while not rospy.is_shutdown():
        limit_switch_states = [ i.get_state() for i in limit_switches]
        rospy.loginfo(limit_switch_states)

        if limit_switch_states[0] and limit_switch_states[1]:
            reset_y_value = Float32(10.0-initial_value["init_y"])
            odom_sety_pub.publish(reset_y_value)
        if limit_switch_states[2] and limit_switch_states[3]:
            reset_y_value = Float32(initial_value["init_y"])
            odom_sety_pub.publish(reset_y_value)
        if limit_switch_states[4] and limit_switch_states[5]:
            reset_x_value = Float32(initial_value["init_x"])
            odom_setx_pub.publish(reset_x_value) 

        rate.sleep()
