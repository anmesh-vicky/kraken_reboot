#package name "Mission_Planner"
import roslib; roslib.load_manifest('smach_planner')
import rospy
import smach 
import smach_ros
import actionlib

import smach_planner.msg

# defining depth with two out comes depth_success , depth_fail ,server to be hardcoded

class Depth(smach.State):
	def _init_(self):
		smach.State._inti_(self,outcome=['depth_success',"depth_fail"])


	def execute(self):
		client= actionlib.SimpleActionClient(<server name>, smach_planner.msg.smach_plannerAction)
    	client.wait_for_server()
    	goal = smach_planner.msg.smach_plannerGoal
    	goal.depth=5
    	client.send_goal(goal)
    	client.wait_for_result()
    	result=client.get_result()  
    	if result ==1
    		return 'depth_success'
    	else:
    		return 'depth_fail'


#defining Ip_Search with gate_search 
class Ip_Search_gate(smach.State):
	def _init_(self):
		smach.State._init_(self,outcome=['Gate_Passed','Gate_Failed'])

	def execute(self):
	client= actionlib.SimpleActionClient(<server name>, smach_planner.msg.smach_plannerAction)
    client.wait_for_server()
    goal = Mission_Planner.msg.smach_plannerGoal



    goal.value=1
    client.send_goal(goal,feedback_cb=Feedback_gate)
    # using feedback to execute the surge and activating the controls



    def Feedback_gate(self,feedback):
    	if feedback == 1:
    		# <surge call surge funtion to some extend>
    	elif feedback ==2:
    		# calls control server to execute its path by taking data from ip server after task is completed gives result/feedback
    	else
    	# yet to be decided





    client.wait_for_result()
    result=client.get_result()  
    if result ==
    	return 'Gate_Passed'
    else:
    	return 'Gate_failed'

    #take feed back and call new server to surge toward goal
    #or is not detect fter some time <timer should be implemented>
    #cancel goal and return some value to start some surge 

    #indicator to be made when first task is completed
    #
  



class Ip_Search_drum(smach.State):
	def _init_(self):
		smach.State._init_(self,outcome=[""])
subl

	def execute(self):
		client= actionlib.SimpleActionClient(<server name>, smach_planner.msg.smach_plannerAction)	
    	client.wait_for_server()
    	goal = Mission_Planner.msg.smach_plannerGoal



    	goal.value=2	
    	client.send_goal(goal,feedback_cb=Feedback_drum)
    # using feedback to execute the surge and activating the controls



    	def Feedback_drum(self,feedback):
    	if feedback == 1:
    		# <surge call surge funtion to some extend>
    	elif feedback ==2:
    		# calls control server to execute its path by taking data from ip server after task is completed gives result/feedback
    	else
    	# yet to be decided





    client.wait_for_result()
    result=client.get_result()  
    if result ==
    	return 'Drum_Passed'
    else:
    	return 'Drum_failed'

    #take feed back and call new server to surge toward goal
    #or is not detect fter some time <timer should be implemented>
    #cancel goal and return some value to start some surge 

   
def main():
    rospy.init_node('mission_planner')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Depth', Depth(), 
                               transitions={'depth_success':'Ipsearch_Gate', 
                                            'depth_fail':'outcome5'})
        smach.StateMachine.add('IpSearch_gate', Ip_Search_gate(), 
                               transitions={'Gate_Passed':'IpSearch_drum', 
                                            'Gate_Failed':'outcome5'})
       

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()









