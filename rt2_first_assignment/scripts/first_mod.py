#! /usr/bin/env python

"""
.. module:: first_mod_node
    :platform: Unix
    :synopsis: Python module that implements under request the first control modality

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

This node implements modality 1 under request of the notebook module named 'rt2_assignment1'. Specifically, it receives the target positions sent by the notebook and it sets them. 
After waiting for the fulfilment of the reaching task for a certain amount of time (60s), it communicates back to the notebook the final status of the goal (either reached or not reached).

Subscribes to:
    - /set_goal

Publishes to:
    - /goal_status

Client:
    - /change_mod1

Action client:
    - /move_base

"""

import rospy # import rospy to use ros functionalities
from std_msgs.msg import Bool
from rt2_first_assignment.msg import SetGoal
from rt2_first_assignment.srv import ChangeMod, ChangeModResponse #import both the request message type and the response message type of the 'ChangeMod.srv' custom service
import actionlib # import actionlib to use actions
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #import both the MoveBaseAction and MoveBaseGoal type of action message

# Action Client
action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) #initialize and define the client that sends requests belonging to the '/move_base' action
"""
Global action client for both setting and cancelling a target position
"""

# Publisher
pub = rospy.Publisher('/goal_status', Bool, queue_size=1) #initialize and define the publisher that publishes on the '/goal_status' topic
"""
Global publisher for warning the notebook node about the goal final status
"""

# Published message
status = Bool()
"""
Global message published on the '/goal_status' topic (contains the final status of the current goal)
"""

# GLOBAL VARIABLES

required_mod = "0"
"""
Global string variable containing the number of the required modality
"""
set_mod = "0"
"""
Global string variable containing the number of the set modality
"""

# SERVICES CALL-BACK FUNCTIONS

# function that is called every time that a new client request related to the /change_mod service is received
def clbk_changemod_srv1(req):

    """Function that is called every time that a new-client request related to the '/change_mod1' service is received. 

    The number related to the required modality is stored in a global variable as a string. If the string is "0" the corresponding modality is set and the robot is stopped.
    Otherwise the required modality is set only if the previous modality was "0" (that is: only if the user went back to the main menu).

    Args:
        req (str): number of the desired control modality stored as a string

    Returns:
        True: boolean constant to warn the client about the fact that the modality has been set
        False: boolean constant to warn the client about the fact that the modality has not been set

    """

    global required_mod
    global set_mod

    required_mod = req.modality
    #print("the requested modality is: " + required_mod)
    if(required_mod == "0"):
        set_mod=required_mod
        return ChangeModResponse(True)
    else: #store in the previous modality variable the required modality only if the previous modality was 0 (aka only if I went back to the main menu)
        if(set_mod == "0"):
            set_mod=required_mod
            return ChangeModResponse(True)
        else:
            set_mod=set_mod
            print('\033[91m' + "\nWarning" + '\033[0m' + ': return to modality selection to change the modality')
            return ChangeModResponse(False)

# AUXILIARY FUNCTIONS

def set_goal_position(x, y):

    """Function that is called in order to send requests belonging to the '/move_base' action.

    First the goal position passed as argument is sent in form of request to the action server. Then the process waits for the fulfilment of the task for a certain amount of time (60s).
    If the action server provides a result, the information about the successful reaching of the target position is published on the '/goal_status' topic. 
    Otherwise the task is cancelled via another request to the action server and the failure is communicated by means of the '/goal_status' topic.

    Args:
        x (float): x-coordinate of the target position
        y (float): y-coordinate of the target position

    """
    global action_client
    global pub

    action_client.wait_for_server()

    goal= MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1.0 
    goal.target_pose.pose.position.x= x
    goal.target_pose.pose.position.y= y

    action_client.send_goal(goal) #send the goal message to the action server 

    print('\033[93m' + "\nDriving towards the goal..." + '\033[0m')

    finished_before_timeout = action_client.wait_for_result(rospy.Duration.from_sec(60.0)) # wait 60 seconds for the server to finish performing the action
    
    if (finished_before_timeout): #if the result is output before the elapsing of the timer, the robot succesfully reached the target position
        print('Goal reaching:' + '\033[92m' + ' SUCCEDED' + '\033[0m')
        status.data = True
        pub.publish(status)
    else: #if the timer elapses, the robot failed in reaching the target position
        print('Goal reaching:' + '\033[91m' + ' FAILED (timeout)' + '\033[0m')
        action_client.cancel_goal()
        status.data = False
        pub.publish(status)

# SUBSCRIBERS CALL-BACK FUNCTIONS

def clbk_setgoal(msg):
    """Function that is called every time that a new message is published on the '/set_goal' topic.

    If the set modality is 1, the goal position contained in the message is forwarded to the 'set_goal_position' function; otherwise a warning message is printed on the terminal.

    Args:
        msg (struct): structure containing the coordinates of the goal position

    """
    global set_mod
    
    if (set_mod == '1'):
        goal_x=msg.abscissa
        goal_y=msg.ordinate
        set_goal_position(goal_x,goal_y)
    else:
        print('\033[91m' + "\nWarning" + '\033[0m' + ": choose or change the modality to make the robot reach a goal position")



# MAIN FUNCTION

def main():

    """Function that first initializes and defines the subscriber and the service, and then simply spins to allow the call-back functions to be called whenever a message arrives on the 
    correspondent communication channel.
    """

    rospy.init_node('first_mod_node') # initialize the node with the name 'first_mod_node'

    sub_setgoal = rospy.Subscriber('/set_goal', SetGoal, clbk_setgoal)  # initialize and define the subscriber that subscribes to the '/set_goal' topic and assign the 'clbk_setgoal' call-back function to it 
    changemod_srv1 =rospy.Service('/change_mod1', ChangeMod, clbk_changemod_srv1) # initialize and define the server that answers to requests belonging to the '/change_mod1' service and assign the 'clbk_changemod_srv1' call-back function to it
    
    rospy.spin() # spin to allow the call-back functions to be called whenever a message arrives on the correspondent topic or service




if __name__ == '__main__': # if this node is run directly:
    main()