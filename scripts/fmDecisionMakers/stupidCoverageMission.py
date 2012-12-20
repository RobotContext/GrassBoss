#!/usr/bin/env python
import roslib; roslib.load_manifest("GrassBoss")
import rospy
import smach
import smach_ros
import actionlib
from std_msgs.msg import Float64
import stupidCoverageBehaviour

# Not used
class saveData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dataSaved'])

    def execute (self, userdata):
        rospy.sleep(10)
        return 'dataSaved'

# Not used
class resetCovered(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['resetDone'])

    def execute (self, userdata):
        rospy.sleep(10)
        return 'resetDone'

def missionState_cb(userdata, msg):
    if (msg.data > 0.9): # Percentage defined here <<< 
        return False
    else:
        return True

def child_term_cb(outcome_map):
    return True

def build_sm():
    top_sm = smach.StateMachine(outcomes=['allDone'])
    with top_sm:
        sm_con = smach.Concurrence(outcomes=['missionDone'], 
                                   default_outcome='missionDone',
                                   child_termination_cb = child_term_cb,
                                   outcome_map={'missionDone':{'missionState':'invalid'}})
        with sm_con:
            behavioursm = stupidCoverageBehaviour.build_stupidBehaviour_sm()
            smach.Concurrence.add('behaviour', behavioursm)
            smach.Concurrence.add('missionState', smach_ros.MonitorState("/mapInterface/coverage", Float64, missionState_cb))
        smach.StateMachine.add('mission', sm_con, transitions={'missionDone':'allDone'})
        #smach.StateMachine.add('saveData', saveData(), transitions={'dataSaved':'resetCovered'})
        #smach.StateMachine.add('resetCovered', resetCovered(), transitions={'resetDone':'allDone'})
        
    return top_sm


def main():
    rospy.init_node("stupidCoverageMission")
    sm = build_sm()
    sis = smach_ros.IntrospectionServer('server_name', sm, '/STATE MACHINE')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
