#!/usr/bin/env python
import rospy
import sys
import numpy as np
from modular import Module
from nav_msgs.msg import OccupancyGrid 
from geometry_msgs.msg import PolygonStamped
import actionlib
from std_msgs import msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from matplotlib import pyplot as plt
import matplotlib

RADIUS = 5
VISITED_COLOR = 50

class RoomInspectorModule(Module):
    def __init__(self, agent_id, room_map):
        super(RoomInspectorModule, self).__init__(agent_id, 'RoomInspectorModule')

        self.cli_cmds = ['v', 'visited_map', 'r', 'room']
        self.loc_sub = rospy.Subscriber(self.get_topic('/move_base/local_costmap/footprint'), PolygonStamped, update_position, self)
        self.print_v(self.get_topic('/move_base/local_costmap/footprint'))
        self.print_v("Setting up room inspector module...")
        map_msg = rospy.wait_for_message(self.get_topic('/map'), OccupancyGrid, timeout=None)
        self.room_map = room_map
        self.client = actionlib.SimpleActionClient('/tb3_%d/move_base'%self.agent_id, MoveBaseAction) # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client.wait_for_server() # Waits until the action server has started up and started listening for goals.
        
        # == init_map == #
        self.visited_map = (np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)) != 0) * 100
        self.visited_map = np.fliplr(np.rot90(self.visited_map))
        self.map_height = self.visited_map.shape[0] # NOTE - this may be revered, as in map_height=shape[1] etc.
        self.map_width = self.visited_map.shape[1]
        self.starting_unexplored = (self.visited_map == 0).sum()

        self.percent_array = np.zeros((1,2))
        self.start_time = rospy.get_rostime().secs
        self.is_moving = False
        self.verbose = True
        self.print_v("Finished setting up room inspector module")


    def update(self):
        if not self.is_moving:
            self.is_moving = True
            x, y = self.sample_unvisited_point_in_room()
            x, y = self.map_to_footprint(x, y)
            print('Moving to X:', x, 'Y:', y)
            self.movebase_client(x, y)

    def sample_unvisited_point_in_room(self):
        x = np.random.  randint(self.map_width)
        y = np.random.randint(self.map_height)
        while self.visited_map[y, x] != VISITED_COLOR and self.room_map[y, x] != True:
            x = np.random.randint(self.map_width)
            y = np.random.randint(self.map_height)
        return x, y

    def cli(self, cmd):
        if cmd in ['v', 'visited_map']:
            plt.imshow(self.visited_map, cmap='gray')
            plt.show()
        if cmd in ['r', 'room']:
            plt.imshow(self.room_map, cmap='gray')
            plt.show()

    def footprint_to_map(self, x, y):
        map_y = self.map_width - int((x + 10) * 20)
        map_x = self.map_height - int((y + 10) * 20)
        return (map_x, map_y)

    def map_to_footprint(self, x, y):
        footprint_y = ((self.map_height - x) / 20) - 10
        footprint_x = ((self.map_width - y) / 20) - 10
        return (footprint_x, footprint_y)

    def movebase_client(self, x, y):
        print("moving to: " + str(x) + "," + str(y))
        goal = MoveBaseGoal() # Creates a new goal with the MoveBaseGoal constructor
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0 # No rotation of the mobile base frame w.r.t. map frame
        self.client.send_goal(goal, done_cb=self.goal_finished) # Sends the goal to the action server.
        rospy.loginfo("New goal command sent!")

    def goal_finished(self, state, result):
        """
        Gets called after the move_base finished, both in case of sucess and failure
        """
        if self.verbose:
            rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))
        self.is_moving = False

def update_position(msg, mod):
    center_x = sum([p.x for p in msg.polygon.points]) / len(msg.polygon.points)
    center_y = sum([p.y for p in msg.polygon.points]) / len(msg.polygon.points)
    
    x,y = mod.footprint_to_map(center_x,center_y)

    mod.current_x = x
    mod.current_y = y

    low_x = max(0, x - RADIUS)
    high_x = min(mod.map_width, x + RADIUS)
    low_y = max(0, y - RADIUS)
    high_y = min(mod.map_height, y + RADIUS)

    new_covered = np.zeros_like(mod.visited_map)
    new_covered[low_y: high_y, low_x: high_x] = VISITED_COLOR
    mod.visited_map = np.maximum(mod.visited_map, new_covered)
    unexplored = (mod.visited_map == 0).sum()
    mod.percentage = (mod.starting_unexplored - unexplored) * 100 / mod.starting_unexplored
    mod.percent_array = np.append(mod.percent_array, np.array([[rospy.get_rostime().secs - mod.start_time, mod.percentage]]), axis=0)