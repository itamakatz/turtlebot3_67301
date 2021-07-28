#!/usr/bin/env python
import sys
import rospy
import os
import select
from collections import defaultdict
import multi_move_base 
import actionlib
from math import sqrt
from visited_map_module import VisitedMapModule
from std_msgs import msg
from roomba_module import RoombaModule
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PolygonStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from count_module import CountModule
from base_collector_module import BaseCollectorModule
import numpy as np
from sklearn.cluster import KMeans
from cv2 import floodFill
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn.neighbors import NearestNeighbors
from sklearn.neighbors import kneighbors_graph
from geometry_msgs.msg import Point

from utils import get_position, get_dirt_distances, generate_dirt, movebase_client, get_dirt_list

# Parameters Optimized for the default balls Gazebo in default map - Needs to be adjusted for different size:
COSTMAP_THRESHOLD = 0.3
PIXEL_THRESHOLD = 50

ROOM_NUMBER = 2
ROOM_COLORS = ['white', 'red', 'blue']

MAX_DIRT_DISTANCE = 2
RADIUS = 5
VISITED_COLOR = 45

"""
General Util Code:
"""

class Module(object):

    def __init__(self, agent_id, name):
        self.cli_cmds = []
        self.agent_id = agent_id
        self.base_topic = '/tb3_' + str(agent_id)
        self.name = name
        self.verbose_name = self.name + '.' + str(self.agent_id) + ' | '

    def update(self): return None

    def cli(self, cmd): pass

    def print_v(self, msg, newline = False): 
        '''verbose print'''
        if(newline): print(self.verbose_name); print(msg)
        else: print(self.verbose_name + str(msg))
    
    def get_topic(self, topic): 
        # self.print_v("composed topic: '" + self.base_topic + topic + "'")
        return self.base_topic + topic

    def get_other_id(self): 
        '''returns the id of the other agent'''
        return abs(self.agent_id-1)

class Modular:

    def __init__(self, module_list):
        self.modules = module_list
        self.cli_cmds = defaultdict(list)
        for module in self.modules:
            for cmd in module.cli_cmds:
                self.cli_cmds[cmd].append(module)
           
    def run(self):
        # Update the modules
        while (not rospy.is_shutdown()):
            self.run_single_round()   

    # Needed for the parallels runs:
    def run_single_round(self):
        for module in self.modules: 
            optional_msg = module.update()
            if(optional_msg is not None): self.parse_cli(optional_msg)

        # Parse cli commands
        has_input, _, _ = select.select( [sys.stdin], [], [], 0.1 )

        if (has_input):
            cli_str = sys.stdin.readline().strip().lower()
            print("got cli command: '" + cli_str + "'")
            self.parse_cli(cli_str)

    def parse_cli(self, msg):
        if msg in ['k', 'kill']: 
            print("killing program")
            sys.exit()
        for module in self.cli_cmds[msg]: 
            module.cli(msg)

"""
Code for Cleaning:
"""

class AdvancedCollectorModule(Module):

    def __init__(self, agent_id):
        super(AdvancedCollectorModule, self).__init__(agent_id, 'AdvancedCollectorModule')
        self.cli_cmds = []
        self.finish = False
        # self.opponent_vect = None
        # self.opponent_last_position = None
        # dirt = rospy.wait_for_message("/dirt",msg.String, 1)
        # dirt_list = np.array(eval(dirt.data))
        # self.print_v(dirt_list,True)
        # # X = np.array([[-1, -1], [-2, -1], [-3, -2], [1, 1], [2, 1], [3, 2]])
        # X = np.array([[-1, -1], [3, 2]])
        # nbrs = NearestNeighbors(n_neighbors=len(dirt_list), algorithm='ball_tree').fit(dirt_list)
        # distances, indices = nbrs.kneighbors(X)
        # self.print_v(indices,True)
        # self.print_v(distances,True)
        # self.print_v(nbrs.kneighbors_graph(X).toarray(),True)
        
        # # plt.scatter(dirt_list[:,0], dirt_list[:,1], c='#00FF00', label="dirt_list")
        # # plt.scatter(X[:,0], X[:,1], c='#FFFF00', label="X")
        # # plt.show()

        # lengths1 = get_dirt_distances(self.agent_id)
        # lengths1
        # self.print_v(lengths1)
        # lengths2 = get_dirt_distances(self.other_agent_id)
        # self.print_v(lengths2)

    def update(self): 
        current_p = get_position(self.agent_id)
        self.print_v("Current location: " + str(current_p.x) + "," + str(current_p.y))
        dirt_distances = get_dirt_distances(self.agent_id)
        opponent_dirt_distances = get_dirt_distances(self.agent_id) # read it here so we dont pick dirt in between giving an error

        if(not opponent_dirt_distances and not self.finish):
            self.finish = True
            print("no more dirt")
            return 'v'
        elif(not opponent_dirt_distances):
            return

        opponent_p1 = get_position(self.other_agent_id)
        rospy.sleep(1)
        opponent_p2 = get_position(self.other_agent_id)

        opponent_vect = Point()
        opponent_vect.x = opponent_p2.x - opponent_p1.x
        opponent_vect.y = opponent_p2.y - opponent_p1.y
        
        self.print_v("opponent_vect: " + str(opponent_vect))

        if(AdvancedCollectorModule.vec_magnitude(opponent_vect) > 0):
            dirt_list = get_dirt_list()
            dirt_distance_from_line = list(map(lambda dirt: (AdvancedCollectorModule.distance_from_line(opponent_p1, opponent_p2, dirt), dirt), dirt_list))
            self.print_v(dirt_distance_from_line, True)
            closest_pair = min(dirt_distance_from_line, key=lambda pair: pair[0])
            if(closest_pair < MAX_DIRT_DISTANCE and dirt_distances(closest_pair[1]) < opponent_dirt_distances(closest_pair[1])):
                goto_dirt = closest_pair[1]
                self.print_v("closest dirt to line: (" + str(goto_dirt[0]) + "," + str(goto_dirt[1]) + ")")
            else:
                goto_dirt = min(dirt_distances, key=dirt_distances.get)
        else:
            goto_dirt = min(dirt_distances, key=dirt_distances.get)                

        movebase_client(self.agent_id, goto_dirt[0], goto_dirt[1])

    @staticmethod
    def vec_magnitude(vec): 
        if(vec is None): return 0
        return sqrt(vec.x**2 + vec.y**2)

    @staticmethod
    def distance_from_line(p1, p2, p0):
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        x0, y0 = p0[0], p0[1]
        return abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/sqrt((x2-x1)**2+(y2-y1)**2)

def vacuum_cleaning(agent_id):
       
    print('start cleaning')
    # robot = Modular([
    #     BaseCollectorModule(agent_id),
    #     # VisitedMapModule(agent_id)
    # ])
    
    if(agent_id == 1):
        robot = Modular([
            BaseCollectorModule(agent_id)
        ])
    elif(agent_id == 0):
        robot = Modular([
            AdvancedCollectorModule(agent_id)
        ])
    else:
        raise NotImplementedError

    # robot1 = Modular([
    #     BaseCollectorModule(0),
    # ])
    # robot2 = Modular([
    #     BaseCollectorModule(1),
    # ])

    # while(not rospy.is_shutdown()):
    #     robot1.run_single_round()
    #     robot2.run_single_round()

    # print("Running modular robot " + str(agent_id))
    robot.run()

"""
Inspection code:
"""
def inspection():
    inspection_controller = InspectionController()
    inspection_controller.run()
    print('start inspection')

class InspectionController:
    def __init__(self):
        self.robot0 = Modular([
            RoomInspectorModule(0),
        ])
        self.robot1 = Modular([
            RoomInspectorModule(1),
        ])

        room_allocation = self.make_rooms()
        self.robot0.modules[0].set_room(self.room_map == room_allocation[0])
        self.robot1.modules[0].set_room(self.room_map == room_allocation[1])
        self.shapes = -2 # Default Number
        print("Inspection Controller setup finished")

    def run(self):
        while(not rospy.is_shutdown()):
            self.robot0.run_single_round()
            self.robot1.run_single_round()
            self.count_shapes()
            print("Number of shapes: ", self.shapes)

    def make_rooms(self):
        map_msg = rospy.wait_for_message('/tb3_0/map', OccupancyGrid, timeout=None)
        map = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)) != 0
        points_x, points_y = np.where(map == False)
        points = np.dstack([points_x, points_y])[0, :, :]
        kmeans = KMeans(n_clusters=ROOM_NUMBER, random_state=0).fit(points)
        
        coloring = kmeans.predict(points)
        
        room_map = np.zeros(map.shape)

        for i in range(points.shape[0]):
            room_map[points[i, 0], points[i, 1]] = coloring[i] + 1

        self.room_map = np.fliplr(np.rot90(room_map))
        # tell imshow about color map so that only set colors are used
        # plt.imshow(room_map, cmap=ListedColormap(ROOM_COLORS))
        # plt.show()
        return self.allocate_rooms()

    def allocate_rooms(self):
        pos0 = (self.robot0.modules[0].current_x, self.robot0.modules[0].current_y)
        pos1 = (self.robot1.modules[0].current_x, self.robot1.modules[0].current_y)
        
        # Calculate centers of rooms:
        centers = []
        for i in range(2):
            x, y = np.where(self.room_map == i+1) # We skip an index since 0 represents out of area
            x_com = x.sum() / x.shape[0]
            y_com = y.sum() / y.shape[0]
            centers.append((x_com, y_com))

        # L_2^2 Norm:
        l22 = lambda pos1, pos2: (pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2
        # print("Centers is ", centers)
        # print("pos0 is ", pos0)
        # print("pos1 is ", pos1)
        if l22(pos0, centers[0]) + l22(pos1, centers[1]) < l22(pos1, centers[0]) + l22(pos0, centers[1]):
            return {0: 1, 1: 2} # For robot0 room number 1 etc.
        else: 
            return {0: 2, 1: 1}

    # Like make_rooms, but from visited maps instead of the regular map
    def make_dynamic_room(non_visited_map1, non_visited_map2):
        # Legal non visited
        pass

    def count_shapes(self):
        CURRENT_COLOR = 25
        PASSED_COLOR = 50
        OBS_COLOR = 100
        # merge the costmaps:
        costmap1 = self.robot0.modules[0].global_costmap
        costmap2 = self.robot1.modules[0].global_costmap

        costmap = np.maximum(costmap1, costmap2)
        thre_map = ((costmap.astype(np.float) / 255) > COSTMAP_THRESHOLD).astype(np.float)
        thre_map = (thre_map*100).astype(np.uint8)
        count = -1 # The first one is the outer walls, two robots
        # First nonempty index:
        x_i, y_i = np.where(thre_map == OBS_COLOR)
        while x_i.shape[0] != 0:
            x = x_i[0]
            y = y_i[0]
            # Fill it
            floodFill(thre_map, None, (y, x), CURRENT_COLOR)
            # if we colored more than PIXEL_THRESHOLD pixels add one to the count
            colored_pixels = (thre_map == CURRENT_COLOR).sum()
            if colored_pixels > PIXEL_THRESHOLD:
                count += 1
            # Return them to a different color
            thre_map[np.where(thre_map == CURRENT_COLOR)] = PASSED_COLOR
            x_i, y_i = np.where(thre_map == OBS_COLOR)
        if self.shapes != -5 and self.shapes != count:
            plt.imshow(thre_map, cmap='gray')
            plt.show()
        self.shapes = count

class RoomInspectorModule(Module):
    def __init__(self, agent_id):
        super(RoomInspectorModule, self).__init__(agent_id, 'RoomInspectorModule')

        self.cli_cmds = ['v', 'visited_map', 'r', 'room']
        self.loc_sub = rospy.Subscriber(self.get_topic('/move_base/local_costmap/footprint'), PolygonStamped, update_position, self)
        self.print_v(self.get_topic('/move_base/local_costmap/footprint'))
        self.print_v("Setting up room inspector module...")
        map_msg = rospy.wait_for_message(self.get_topic('/map'), OccupancyGrid, timeout=None)
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

    def set_room(self, room):
        self.room_map = room

    def update(self):
        # Update the Costmap:
        map_msg = rospy.wait_for_message(self.get_topic('/move_base/global_costmap/costmap'), OccupancyGrid, timeout=None)
        self.global_costmap = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height))    
        
        # Handle the moving:
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

# Used as a callback:
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


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    
    exec_mode = sys.argv[1] 
    print('exec_mode:' + exec_mode)        

    if exec_mode == 'cleaning':        
        agent_id = int(sys.argv[2])
        rospy.init_node('assignment_2.%d'%agent_id)
        vacuum_cleaning(agent_id)
        print('agent id:' + sys.argv[2])        
    elif exec_mode == 'inspection':
        rospy.init_node('assignment_2')
        inspection()
    elif exec_mode == 'start':
        rospy.init_node('starting')
        agent_id = int(sys.argv[2])
        movebase_client(agent_id,-5,-2)
    elif exec_mode == 'dirt':
        rospy.init_node('dirt')
        generate_dirt()
    else:
        print("Code not found")
        raise NotImplementedError