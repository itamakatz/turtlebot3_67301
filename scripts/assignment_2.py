#!/usr/bin/env python
import sys
import rospy
import os
import multi_move_base 
import actionlib
from visited_map_module import VisitedMapModule
from std_msgs import msg
from roomba_module import RoombaModule
from nav_msgs.msg import OccupancyGrid, MapMetaData
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from modular import Module, Modular
from geometry_msgs.msg import PoseWithCovarianceStamped
from count_module import CountModule
from base_collector_module import BaseCollectorModule


# class BaseCollectorModule(Module):

#     def __init__(self, agent_id):
#         super(BaseCollectorModule, self).__init__(agent_id, 'BaseCollectorModule')
#         self.cli_cmds = []

#     def update(self): 

#         location = rospy.wait_for_message(self.get_topic('/amcl_pose'), PoseWithCovarianceStamped, 1)
#         p = location.pose.pose.position
#         self.print_v("Current location: " + str(p.x) + "," + str(p.y))

#         try:
#             dirt = rospy.wait_for_message("/dirt",msg.String, 1)
#             dirt_list = eval(dirt.data)
#             self.print_v("received: " + dirt.data)
#         except rospy.ROSException:
#             self.print_v("no dirt left")
#             return

#         if(len(dirt_list) == 0):
#             self.print_v("no dirt left")
#             return

#         self.print_v("next dirt location: " + str(dirt_list[0][0]) + "," + str(dirt_list[0][1]))

#         movebase_client(self.agent_id, dirt_list[0][0], dirt_list[0][1]) 

# def movebase_client(agent_id,x,y):

#     print("moving to: " + str(x) + "," + str(y))

#     client = actionlib.SimpleActionClient('/tb3_%d/move_base'%agent_id,MoveBaseAction) # Create an action client called "move_base" with action definition file "MoveBaseAction"
#     client.wait_for_server() # Waits until the action server has started up and started listening for goals.
#     goal = MoveBaseGoal() # Creates a new goal with the MoveBaseGoal constructor
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = x
#     goal.target_pose.pose.position.y = y
#     goal.target_pose.pose.orientation.w = 1.0 # No rotation of the mobile base frame w.r.t. map frame
#     client.send_goal(goal) # Sends the goal to the action server.
#     rospy.loginfo("New goal command received!")
#     wait = client.wait_for_result() # Waits for the server to finish performing the action.
   
#     if not wait:
#         # If the result doesn't arrive, assume the Server is not available
#         rospy.logerr("Action server not available!")
#         rospy.signal_shutdown("Action server not available!")
#     else:
#         print("finished!")
#         return client.get_result()   # Result of executing the action

def vacuum_cleaning(agent_id):
       
    print('start cleaning')
    robot = Modular([
        BaseCollectorModule(agent_id)
    ])
    print("Running modular robot " + str(agent_id))
    robot.run()

def print_lacation(agent_id):
    mMap_mdata = rospy.wait_for_message('/tb3_%d/map_metadata'%agent_id,MapMetaData)
    location = rospy.wait_for_message('/tb3_%d/amcl_pose'%agent_id, PoseWithCovarianceStamped)
    p = location.pose.pose.position
    x_id = int((p.x - mMap_mdata.origin.position.x)/mMap_mdata.resolution)
    y_id = int((p.y - mMap_mdata.origin.position.y)/mMap_mdata.resolution)
    print("Current location: " + str(p.x) + "," + str(p.y))
    print("Current location in the map: " + str(x_id) + "," + str(y_id))

def inspection():
    print('start inspection')
    id = 0
    robot = Modular([
        VisitedMapModule(id),
        CountModule(id),
        RoombaModule(id)
    ])
    print("Running modular robot " + str(id))
    robot.run()


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('assignment_2')

    exec_mode = sys.argv[1] 
    print('exec_mode:' + exec_mode)        
    # id = int(sys.argv[2])
    # print_lacation(id)
    # x = 1.3
    # y = -3.2
    # result = movebase_client(id,x,y)
    # print_lacation(id)

    if exec_mode == 'cleaning':        
        agent_id = int(sys.argv[2])
        vacuum_cleaning(agent_id)
        print('agent id:' + sys.argv[2])        
    elif exec_mode == 'inspection':
        inspection()
    else:
        print("Code not found")
        raise NotImplementedError