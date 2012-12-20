#!/usr/bin/env python
import roslib; roslib.load_manifest("GrassBoss")
import rospy
import smach
import smach_ros
import actionlib
import time

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

class startTimer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['timerStarted'], output_keys=['starCenterTime'])
    def execute (self, userdata):
        userdata.starCenterTime = rospy.Time.now()
        return 'timerStarted'
        

class stopTimer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['timerStopped'], output_keys=['starOutTime'], input_keys=['starCenterTime'])

    def execute(self, userdata):
        t = rospy.Time.now() - userdata.starCenterTime
        userdata.starOutTime = t.secs
        rospy.loginfo(t.secs)
        return 'timerStopped'


class timedCountdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['timerExpired'], input_keys=['starOutTime','test'])
    def execute(self, userdata):
        rospy.sleep(userdata.starOutTime)
        rospy.loginfo(userdata.test)
        return 'timerExpired'

class timedTurn(smach.State):
    def __init__(self, driveActionClient, time):
        smach.State.__init__(self, outcomes=['timedTurnDone'])
        self.turntime = time
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 0
        goal.angle = 1
        self.driveActionClient.send_goal(goal)
        rospy.sleep(self.turntime)
        goal.angle = 0
        self.driveActionClient.send_goal(goal)
        return 'timedTurnDone'

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

def lookForWire_cb(ud, msg):
    if msg.angle != 0:
        rospy.loginfo("found wire")
        return False
    else:
        return True

def center_child_term_cb(outcome_map):
    return True
        
def build_simpleBehaviour_sm():
    driveActionClient = actionlib.SimpleActionClient('constantVelocityAction', constantVelocityAction)
    driveActionClient.wait_for_server()
    sm = smach.StateMachine(outcomes=['DONE','preempted'])
    sm.userdata.starOutTime = 4
    sm.userdata.starCenterTime = 0
    sm.userdata.test = 7

    drivesm = smach.StateMachine(outcomes=['wireFound','preempted'])
    #drivesm.userdata.starOutTime = 3
    drivesm.userdata.starCenterTime = rospy.Time.now()
    with drivesm:
        smach.StateMachine.add('startDrivingForward', startDriving(driveActionClient), transitions={'startedDriving':'lookForWire'})
        smach.StateMachine.add('lookForWire', smach_ros.MonitorState("/angle", angle, lookForWire_cb), transitions={'invalid':'stopDrivingForward', 'valid':'lookForWire', 'preempted':'stopDrivingForward'})
        smach.StateMachine.add('stopDrivingForward', stopDriving(driveActionClient), transitions={'stoppedDriving':'wireFound'})

    staroutsm = smach.StateMachine(outcomes=['starOutDone','preempted'])
    with staroutsm:
        smach.StateMachine.add('startTimer', startTimer(), transitions={'timerStarted':'continueUntilWire'})
        smach.StateMachine.add('continueUntilWire', drivesm, transitions={'wireFound':'stopTimer', 'preempted':'preempted'})
        smach.StateMachine.add('stopTimer', stopTimer(), transitions={'timerStopped':'starOutDone'})

    starinsm = smach.Concurrence(outcomes=['atStarCenter','newCenter'],
                                 default_outcome='atStarCenter',
                                 child_termination_cb = center_child_term_cb,
                                 input_keys=['starOutTime','test'],
                                 outcome_map={'newCenter':{'driveToCenter':'wireFound'},'atStarCenter':{'timedCountdown':'timerExpired'}}
                                 )
    #starinsm.userdata.starOutTime = 2
    with starinsm:
        smach.Concurrence.add('driveToCenter', drivesm)
        smach.Concurrence.add('timedCountdown', timedCountdown())


    with sm:
        smach.StateMachine.add('starOut', staroutsm, transitions={'starOutDone':'avoidWireReverse', 'preempted':'preempted'}, remapping={'starOutTime':'starOutTime'})
        smach.StateMachine.add('avoidWireReverse', avoidWireReverse(driveActionClient), transitions={'done':'turnAround'})
        smach.StateMachine.add('turnAround', timedTurn(driveActionClient, 3), transitions={'timedTurnDone':'starIn'})
        smach.StateMachine.add('starIn', starinsm, transitions={'atStarCenter':'turnStar', 'newCenter':'offsetCenter'}, remapping={'starInTime':'starInTime'})
        smach.StateMachine.add('turnStar', timedTurn(driveActionClient,1), transitions={'timedTurnDone':'starOut'})
        smach.StateMachine.add('offsetCenter', avoidWireReverse(driveActionClient), transitions={'done':'turnNewCenter'})
        smach.StateMachine.add('turnNewCenter', timedTurn(driveActionClient, 3), transitions={'timedTurnDone':'starOut'})

    return sm

def main():
    try:
        rospy.init_node("simple_coverage_behaviour")
        sm = build_simpleBehaviour_sm()
        sis = smach_ros.IntrospectionServer('server_name', sm, '/STATE MACHINE')
        sis.start()
        sm.execute()
        sis.stop()
    except (rospy.exceptions.ROSInterruptException, smach.exceptions.InvalidUserCodeError):
        pass
        
if __name__ == "__main__":
    main()
        
