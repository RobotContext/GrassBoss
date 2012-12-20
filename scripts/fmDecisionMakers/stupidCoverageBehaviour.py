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

class avoidWireReverse(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['done'])
        self.driveActionClient = driveActionClient
    def execute (self, userdata):
        goal = constantVelocityAction
        goal.velocity = -1
        goal.angle = 0
        self.driveActionClient.send_goal(goal)
        rospy.sleep(1)
        return 'done'
        

class avoidWireTurn(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['avoided'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 0
        goal.angle = 3
        self.driveActionClient.send_goal(goal)
        rospy.sleep(0.5)
        return 'avoided'

def lookForWire_cb(ud, msg):
    if msg.angle != 0:
        rospy.loginfo("found wire")
        return False
    else:
        return True
        
def build_stupidBehaviour_sm():
    driveActionClient = actionlib.SimpleActionClient('constantVelocityAction', constantVelocityAction)
    driveActionClient.wait_for_server()
    sm = smach.StateMachine(outcomes=['DONE','preempted'])
    with sm:
        drivesm = smach.StateMachine(outcomes=['wireFound','preempted'])
        with drivesm:
            smach.StateMachine.add('startDrivingForward', startDriving(driveActionClient), transitions={'startedDriving':'lookForWire'})
            smach.StateMachine.add('lookForWire', smach_ros.MonitorState("/angle", angle, lookForWire_cb), transitions={'invalid':'stopDrivingForward', 'valid':'lookForWire', 'preempted':'preempted'})
            smach.StateMachine.add('stopDrivingForward', stopDriving(driveActionClient), transitions={'stoppedDriving':'wireFound'})
        avoidsm = smach.StateMachine(outcomes=['wireAvoided'])
        with avoidsm:
            smach.StateMachine.add('avoidWireReverse', avoidWireReverse(driveActionClient), transitions={'done':'avoidWireTurn'})
            smach.StateMachine.add('avoidWireTurn', avoidWireTurn(driveActionClient), transitions={'avoided':'wireAvoided'})

        smach.StateMachine.add('continueUntilWire', drivesm, transitions={'wireFound':'avoidWire', 'preempted':'preempted'})
        smach.StateMachine.add('avoidWire', avoidsm, transitions={'wireAvoided':'continueUntilWire'})

    return sm

def main():
    try:
        rospy.init_node("stupid_coverage_behaviour")
        sm = build_stupidBehaviour_sm()
        #sis = smach_ros.IntrospectionServer('server_name', sm, '/STATE MACHINE')
        #sis.start()
        sm.execute()
        #sis.stop()
    except (rospy.exceptions.ROSInterruptException, smach.exceptions.InvalidUserCodeError):
        pass
        
if __name__ == "__main__":
    main()
        
