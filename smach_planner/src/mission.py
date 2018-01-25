#package name "Mission_Planner"
import roslib; roslib.load_manifest('Mission_Planner')
import rospy
import smach 
import smach_ros
import actionlib

import mission_planner.msg


class Depth(smach.State):
	def _init_(self):
		smach.State._inti_(self,outcome=['depth_success',"depth_fail"])


	def execute(self):
		client=actionlib.SimpleAction(<server name>, mission_planner.msg.<msg>)
    client.wait_for_server()
    goal = mission_planner.msg.<goal msg>(order=20)
    client.send_goal(goal)
    client.wait_for_result()
    result=client.get_result()  
    if result ==
    	return 'depth_success'
    else:
    	return 'depth_fail'


class Validation_Gate(smach.State)
	def _init_(self):
		smach.State._inti(self,outcome=['success_gate','fail_gate'])



	def execute(self):
		client=actionlib.SimpleAction(<server name>,mission_planner.msg<msg>)
		client.wait_for_server()
		goal=mission_planner.msg.<goal msg>
		goal.value=1#"1 for validation gate"
		client.send_goal(goal)
		client.wait_for_result()
		result=client.get_result()
		if result=="success"
			return success_gate
		else :
			return fail_gate


class Drum(smach.State)
	def _init_(self):
		smach.State._init(self,outcome=['success_drum',"fail_drum"])

	def execute(self):
		client=actionlib.SimpleAction(<server name>,mission_planner.msg<msg>)
		client.wait_for_server()
		goal=mission_planner.msg.<goal msg>
		goal.value=2#"1 for drum"
		client.send_goal(goal)
		client.wait_for_result()
		result=client.get_result()
		if result=="success"
			return success_drum
		else :
			return fail_drum






class yaw(smach.State)
	def _init_(self):
		smach.State._init(self,outcome=['success_yaw','fail_yaw'])

	
	def execute(self):
		client=actionlib.SimpleAction(<server name>,mission_planner.msg<msg>)
		client.wait_for_server()
		goal.angular_velocity=.2
		client.send_goal(goal)
		## how to stop the control yaw server in concurrent state?
		## subscring to ip client that publish when to stop.
		rospy.init_node(<node name>, anonymous=True)

    	rospy.Subscriber("mission_plannner\result", <type of action msg>, callback)



def main():
    rospy.init_node('mission_planner')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Depth', Depth(), 
                               transitions={'':'', 
                                            '':'outcome5'})
       
        smach.StateMachine.add('', (), 
                               transitions={'':'Depth', 
                                            '':'outcome5'})

    # Execute SMACH plan
    outcome = sm.execute()

#     sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
#     sis.start()

# # Execute the state machine
#     outcome = sm.execute()

# # Wait for ctrl-c to stop the application
#     rospy.spin()
#     sis.stop()


if __name__ == '__main__':
    main()

