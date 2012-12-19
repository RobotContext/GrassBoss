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
        

class timedAngleTurn(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['done'], input_keys=['angle'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 0
        goal.angle = 1
        self.driveActionClient.send_goal(goal)
        rospy.sleep(1)
        return 'done'


class timedDrive(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['done'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 5
        goal.angle = 0

        self.driveActionClient.send_goal(goal)
        rospy.sleep(2)
        goal.velocity = 0
        self.driveActionClient.send_goal(goal)
        return 'done'


def lookForWire_cb(ud, msg):
    if msg.angle != 0:
        rospy.loginfo("found wire")
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
            alignsm = smach.StateMachine(outcomes=['aligned'])
            with alignsm:
                turnsm = smach.StateMachine(outcomes=['turned'])
                turnsm.userdata.angle = 90
                turnsm.userdata.angularspeed = 1

                smach.StateMachine.add('timedAngleTurnIn', timedAngleTurn(driveActionClient), transitions={'done':'timedDrive'})
                smach.StateMachine.add('timedDrive', timedDrive(driveActionClient), transitions={'done':'timedAngleTurnOut'})
                smach.StateMachine.add('timedAngleTurnOut', timedAngleTurn(driveActionClient), transitions={'done':'aligned'})

            smach.StateMachine.add('continueUntilWire', drivesm, transitions={'wireFound':'alignForNextRow'})
            smach.StateMachine.add('alignForNextRow', alignsm, transitions={'aligned':'continueUntilWire'})
        sis = smach_ros.IntrospectionServer('server_name', sm, '/STATE MACHINE')
        sis.start()
        sm.execute()
        sis.stop()
    except (rospy.exceptions.ROSInterruptException, smach.exceptions.InvalidUserCodeError):
        pass
        
if __name__ == "__main__":
    rospy.loginfo("woooo")
    main()
