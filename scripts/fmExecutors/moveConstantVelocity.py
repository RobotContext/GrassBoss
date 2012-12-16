#!/usr/bin/env python

import roslib; roslib.load_manifest('GrassBoss')
import rospy
import actionlib
from geometry_msgs.msg import Twist
from GrassBoss.msg import *

class MoveConstantVelocityAction():
    def __init__(self):
        self.desired_velocity = 0
        self.goal = False

        self.__server = actionlib.SimpleActionServer('constantVelocityAction', constantVelocityAction, auto_start=False)
        self.__server.register_preempt_callback(self.preempt_cb)
        self.__server.register_goal_callback(self.goal_cb)
        
        self.__server.start()


        self.vel_pub = rospy.Publisher("/cmd_vel", Twist)
        
    def preempt_cb(self):
        goal = False
        rospy.loginfo("Goal was preempted")
        self.publish_velocity(0)
        
    def goal_cb(self):
        g = self.__server.accept_new_goal()
        self.desired_velocity = g.velocity
        if g.velocity != 0:
            self.goal = True
        else:
            self.goal = False


    def on_timer(self, e):
        if self.goal:
            self.publish_velocity(self.desired_velocity)

    def publish_velocity(self, newVelocity):
        vel = Twist()
        vel.linear.x = newVelocity
        self.vel_pub.publish(vel)

if __name__ == "__main__":
    try:
        rospy.init_node("moveConstantVelocity")
        action_server = MoveConstantVelocityAction()
        t = rospy.Timer(rospy.Duration(0.05), action_server.on_timer)
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
        
