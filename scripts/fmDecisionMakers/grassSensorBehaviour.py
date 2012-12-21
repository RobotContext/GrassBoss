#!/usr/bin/env python
import roslib; roslib.load_manifest("GrassBoss")
import rospy
import smach
import smach_ros
import actionlib

from GrassBoss import *
from GrassBoss.msg import angle, constantVelocityAction
from std_msgs.msg import Bool

class startDriving(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['startedDriving'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 10
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
        goal.velocity = -5
        goal.angle = 0
        self.driveActionClient.send_goal(goal)
        rospy.sleep(1.0)
        return 'done'
        

class avoidWireTurn(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['avoided'])
        self.driveActionClient = driveActionClient

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 0
        goal.angle = 5
        self.driveActionClient.send_goal(goal)
        rospy.sleep(0.5)
        return 'avoided'

class correctBearing(smach.State):
    def __init__(self, driveActionClient):
        smach.State.__init__(self, outcomes=['corrected'])
        self.driveActionClient = driveActionClient
	self.turn = True

    def execute(self, userdata):
        goal = constantVelocityAction
        goal.velocity = 10
	if self.turn:
        	goal.angle = 1
		self.turn = True
	else:
		goal.angle = -1
		self.turn = True
        self.driveActionClient.send_goal(goal)
        rospy.sleep(0.1)
        return 'corrected'

def lookForWire_cb(ud, msg):
    if msg.angle != 0:
        rospy.loginfo("found wire")
        return False
    else:
        return True

def senseGrass_cb(ud, msg):
    if msg.data:
        return True
    else:
        return False

def force_preempt(a):
    return True

def main():
    try:
        rospy.init_node("continueStraightUntilWire")
        driveActionClient = actionlib.SimpleActionClient('constantVelocityAction', constantVelocityAction)
        driveActionClient.wait_for_server()

        sm = smach.StateMachine(outcomes=['DONE'])
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()
        with sm:
            drivesm = smach.StateMachine(outcomes=['wireFound','outOfGrass'])
            with drivesm:
                smach.StateMachine.add('startDriving', startDriving(driveActionClient), transitions={'startedDriving':'lookForWire'})
		monitorsm = smach.Concurrence(outcomes=['stopDriving','noGrassToCut'],default_outcome='stopDriving',
                           outcome_map={"stopDriving":{'senseWire':'invalid','senseGrass':'preempted'}, 
                                        "stopDriving":{'senseGrass':'invalid','senseWire':'invalid'},"noGrassToCut":{'senseGrass':'invalid','senseWire':'preempted'}}, child_termination_cb=force_preempt)

		with monitorsm:
                    smach.Concurrence.add('senseWire', smach_ros.MonitorState("/angle", angle, lookForWire_cb))
		    smach.Concurrence.add('senseGrass', smach_ros.MonitorState("/grass_sensor", Bool, senseGrass_cb))
                smach.StateMachine.add('lookForWire', monitorsm, transitions={'stopDriving':'stopDriving','noGrassToCut':'outOfGrass'})

                smach.StateMachine.add('stopDriving', stopDriving(driveActionClient), transitions={'stoppedDriving':'wireFound'})
		smach.StateMachine.add('noGrassToCut', stopDriving(driveActionClient), transitions={'stoppedDriving':'outOfGrass'})

            smach.StateMachine.add('continueUntilWire', drivesm, transitions={'wireFound':'avoidWireReverse','outOfGrass':'correctBearing'})
            smach.StateMachine.add('avoidWireReverse', avoidWireReverse(driveActionClient), transitions={'done':'avoidWireTurn'})
            smach.StateMachine.add('avoidWireTurn', avoidWireTurn(driveActionClient), transitions={'avoided':'continueUntilWire'})
            smach.StateMachine.add('correctBearing', correctBearing(driveActionClient), transitions={'corrected':'continueUntilWire'})

        sm.execute()
        rospy.spin()
        sis.stop()
    except rospy.exceptions.ROSInterruptException:
        pass
        
if __name__ == "__main__":
    main()
        
