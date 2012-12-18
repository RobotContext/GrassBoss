#!/usr/bin/env python
import roslib; roslib.load_manifest("GrassBoss")
import rospy
import smach
import smach_ros
import actionlib

from GrassBoss import *
from GrassBoss.msg import angle, constantVelocityAction

class startDriving(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['startedDriving'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 5
        goal.angle = 0
        self.driveActionClient.send_goal(goal)
        return 'startedDriving'

class stopDriving(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['stoppedDriving'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        self.driveActionClient.cancel_all_goals()
        goal = constantVelocityAction
        goal.velocity = 0
        goal.angle = 0
        self.driveActionClient.send_goal(goal)
        return 'stoppedDriving'

def lookForWire_cb(ud, msg):
    if msg.angle != 0:
        rospy.loginfo("found wire")
        return False
    else:
        return True
        

def main():
    try:
        rospy.init_node("continueStraightUntilWire")
        driveActionClient = actionlib.SimpleActionClient('constantVelocityAction', constantVelocityAction)
        driveActionClient.wait_for_server()

        sm = smach.StateMachine(outcomes=['DONE'])
        with sm:
            smach.StateMachine.add('startDriving', startDriving(driveActionClient), transitions={'startedDriving':'lookForWire'})
            smach.StateMachine.add('lookForWire', smach_ros.MonitorState("/angle", angle, lookForWire_cb), transitions={'invalid':'stopDriving', 'valid':'lookForWire', 'preempted':'stopDriving'})
            smach.StateMachine.add('stopDriving', stopDriving(driveActionClient), transitions={'stoppedDriving':'DONE'})
        sm.execute()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
        
if __name__ == "__main__":
    main()
        
