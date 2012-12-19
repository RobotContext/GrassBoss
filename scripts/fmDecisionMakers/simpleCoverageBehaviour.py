#!/usr/bin/env python
import roslib; roslib.load_manifest("GrassBoss")
import rospy
import smach
import smach_ros
import actionlib
import math

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
        

class startTurning(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['turning'], input_keys=['angularspeed'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 0
        goal.angle = userdata.angularspeed
        self.driveActionClient.send_goal(goal)
        return 'turning'


class stopTurning(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['stopped'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 0
        goal.angle = 0
        self.driveActionClient.send_goal(goal)
        return 'stopped'

class goAwayFromWire(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['done'])
        self.driveActionClient = driveActionClient
    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 5
        goal.angle = 0
        self.driveActionClient.send_goal(goal)
        rospy.sleep(1)
        goal.velocity = 0
        self.driveActionClient.send_goal(goal)
        return 'done'

def lookForWire_cb(ud, msg):
    if msg.angle != 0:
        rospy.loginfo("found wire")
        return False
    else:
        return True

def waitForWireAngle_cb(ud, msg):
    #rospy.loginfo(ud.angle)
    rospy.loginfo(msg.angle)
    if math.fabs(90 - math.fabs(msg.angle)) <= 2:
        return False
    else:
        return True


def main():
    try:
        rospy.init_node("simpleCoverageBehaviour")
        driveActionClient = actionlib.SimpleActionClient('constantVelocityAction', constantVelocityAction)
        driveActionClient.wait_for_server()

        sm = smach.StateMachine(outcomes=['DONE'])
        with sm:
            drivesm = smach.StateMachine(outcomes=['wireFound'])
            with drivesm:
                smach.StateMachine.add('startDrivingForward', startDriving(driveActionClient), transitions={'startedDriving':'lookForWire'})
                smach.StateMachine.add('lookForWire', smach_ros.MonitorState("/angle", angle, lookForWire_cb), transitions={'invalid':'stopDrivingForward', 'valid':'lookForWire', 'preempted':'stopDrivingForward'})
                smach.StateMachine.add('stopDrivingForward', stopDriving(driveActionClient), transitions={'stoppedDriving':'wireFound'})
            alignsm = smach.StateMachine(outcomes=['wireAvoided'])
            with alignsm:
                turnsm = smach.StateMachine(outcomes=['turned'])
                turnsm.userdata.angle = 90
                turnsm.userdata.angularspeed = 1
                with turnsm:
                    smach.StateMachine.add('startTurning', startTurning(driveActionClient), transitions={'turning':'waitForWireAngle'},remapping={'angularspeed':'angularspeed'})
                    smach.StateMachine.add('waitForWireAngle', smach_ros.MonitorState("/angle", angle, waitForWireAngle_cb), transitions={'invalid':'stopTurning', 'valid':'waitForWireAngle', 'preempted':'stopTurning'}, remapping={'angle':'angle'})
                    smach.StateMachine.add('stopTurning', stopTurning(driveActionClient), transitions={'stopped':'turned'})

                smach.StateMachine.add('turnToWireAngle', turnsm, transitions={'turned':'goAwayFromWire'})
                smach.StateMachine.add('goAwayFromWire', goAwayFromWire(driveActionClient), transitions={'done':'wireAvoided'})
                #smach.StateMachine.add('followWire', avoidWireReverse(driveActionClient), transitions={'done':'avoidWireTurn'})
                #smach.StateMachine.add('avoidWireTurn', avoidWireTurn(driveActionClient), transitions={'avoided':'wireAvoided'})

            smach.StateMachine.add('continueUntilWire', drivesm, transitions={'wireFound':'alignForNextRow'})
            smach.StateMachine.add('alignForNextRow', alignsm, transitions={'wireAvoided':'continueUntilWire'})
        sis = smach_ros.IntrospectionServer('server_name', sm, '/STATE MACHINE')
        sis.start()
        sm.execute()
        sis.stop()
    except (rospy.exceptions.ROSInterruptException, smach.exceptions.InvalidUserCodeError):
        pass
        
if __name__ == "__main__":
    rospy.loginfo("woooo")
    main()
