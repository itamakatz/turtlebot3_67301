#!/usr/bin/env python
import sys
import rospy
import os
import actionlib
from std_msgs import msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from modular import Module
from geometry_msgs.msg import PoseWithCovarianceStamped

class BaseCollectorModule(Module):

    def __init__(self, agent_id):
        super(BaseCollectorModule, self).__init__(agent_id, 'BaseCollectorModule')
        self.cli_cmds = []

    def update(self): 

        location = rospy.wait_for_message(self.get_topic('/amcl_pose'), PoseWithCovarianceStamped)
        p = location.pose.pose.position
        self.print_v("Current location: " + str(p.x) + "," + str(p.y))

        try:
            dirt = rospy.wait_for_message("/dirt",msg.String, 1)
            dirt_list = eval(dirt.data)
            self.print_v("received: " + dirt.data)
        except rospy.ROSException:
            self.print_v("no dirt left")
            return

        if(len(dirt_list) == 0):
            self.print_v("no dirt left")
            return

        # self.print_v("next dirt location: " + str(dirt_list[0][0]) + "," + str(dirt_list[0][1]))

        movebase_client(self.agent_id, dirt_list[0][0], dirt_list[0][1]) 

def movebase_client(agent_id,x,y):

    print("moving to: " + str(x) + "," + str(y))

    client = actionlib.SimpleActionClient('/tb3_%d/move_base'%agent_id,MoveBaseAction) # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client.wait_for_server() # Waits until the action server has started up and started listening for goals.
    goal = MoveBaseGoal() # Creates a new goal with the MoveBaseGoal constructor
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0 # No rotation of the mobile base frame w.r.t. map frame
    client.send_goal(goal) # Sends the goal to the action server.
    rospy.loginfo("New goal command received!")
    wait = client.wait_for_result() # Waits for the server to finish performing the action.
   
    if not wait:
        # If the result doesn't arrive, assume the Server is not available
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print("finished!")
        return client.get_result()   # Result of executing the action
