#!/usr/bin/env python
import sys
import select
import rospy
import random
import os
import numpy as np
import nav_msgs
import matplotlib.pyplot as plt
import math
import actionlib
from visited_map_module import VisitedMapModule
from std_msgs import msg
from sklearn.neighbors import kneighbors_graph
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import KMeans
from roomba_module import RoombaModule
from nav_msgs.msg import OccupancyGrid, MapMetaData
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from matplotlib.colors import ListedColormap
from math import sqrt
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from cv2 import floodFill
from count_module import CountModule
from collections import defaultdict

# Parameters Optimized for the default balls Gazebo in default map - Needs to be adjusted for different size:
COSTMAP_THRESHOLD = 0.3
PIXEL_THRESHOLD = 50

ROOM_NUMBER = 2
ROOM_COLORS = ['white', 'red', 'blue']

MIN_VECTOR_MAGNITUDE = 1e-2
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

    @property
    def other_agent_id(self):
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


def get_position(agent_id):
    try:
        current_location = rospy.wait_for_message('/tb3_%d/amcl_pose'%agent_id, PoseWithCovarianceStamped, 1)
        return current_location.pose.pose.position
    except rospy.ROSException:
        rospy.logerr("waiting for amcl_pose failed.")
        return

def get_dirt_list():
    try:
        dirt = rospy.wait_for_message("/dirt",msg.String, 1)
        dirt_list = eval(dirt.data)
        dirt_list = [tuple(x) for x in dirt_list]
        return dirt_list
    except rospy.ROSException:
        rospy.logerr("could not read dirt")
        return []

def get_dirt_distances(agent_id, dirt_list, service_invoke_agent_id = None):

    if(service_invoke_agent_id is None): service_invoke_agent_id = agent_id

    # note that only here we use the agent_id and not the service_invoke_agent_id!
    current_position = get_position(agent_id)

    lengths = {}
    for dirt in dirt_list:

        start = PoseStamped()
        start.header.seq = 0
        start.header.frame_id = "/tb3_%d/map"%service_invoke_agent_id
        start.header.stamp = rospy.Time(0)
        start.pose.position.x = current_position.x
        start.pose.position.y = current_position.y

        Goal = PoseStamped()
        Goal.header.seq = 0
        Goal.header.frame_id = "/tb3_%d/map"%service_invoke_agent_id
        Goal.header.stamp = rospy.Time(0)
        Goal.pose.position.x = dirt[0]
        Goal.pose.position.y = dirt[1]

        import nav_msgs.srv as srv
        req = srv.GetPlan()
        # req = nav_msgs.srv.GetPlan()
        req.start = start
        req.goal = Goal
        req.tolerance = .5
        # req.tolerance = 2
        
        get_plan = rospy.ServiceProxy('/tb3_%d/move_base/make_plan'%service_invoke_agent_id, nav_msgs.srv.GetPlan)
        resp = get_plan(req.start, req.goal, req.tolerance)
        
        length = poses_to_length(resp.plan.poses)
        lengths[(dirt[0], dirt[1])] = length

    return lengths
    
def poses_to_length(poses_list):

    length = 0
    for i in range(len(poses_list) - 1):
        position_a_x = poses_list[i].pose.position.x
        position_b_x = poses_list[i+1].pose.position.x
        position_a_y = poses_list[i].pose.position.y
        position_b_y = poses_list[i+1].pose.position.y

        length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
    return length    

def movebase_client(agent_id,x,y,verbose = True):

    if(verbose): print("moving to: " + str(x) + "," + str(y))

    client = actionlib.SimpleActionClient('/tb3_%d/move_base'%agent_id,MoveBaseAction) # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client.wait_for_server() # Waits until the action server has started up and started listening for goals.
    goal = MoveBaseGoal() # Creates a new goal with the MoveBaseGoal constructor
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0 # No rotation of the mobile base frame w.r.t. map frame
    client.send_goal(goal) # Sends the goal to the action server.
    # rospy.loginfo("New goal command received!")
    # wait = client.wait_for_result(rospy.Duration(1)) # Waits for the server to finish performing the action.
    wait = client.wait_for_result() # Waits for the server to finish performing the action.

    if not wait:
        # If the result doesn't arrive, assume the Server is not available
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # print("finished!")
        return client.get_result()   # Result of executing the action


def get_dirt_angles(agent_id, dirt_list):

    try:
        current_pose = rospy.wait_for_message('/tb3_%d/amcl_pose'%agent_id, PoseWithCovarianceStamped, 1)
        current_position = current_pose.pose.pose.position
        current_orientation_radian = np.arccos(current_pose.pose.pose.orientation.z)*2
    except rospy.ROSException:
        rospy.logerr("waiting for amcl_pose failed.")

    current_position = get_position(agent_id)

    angles = {}
    for dirt in dirt_list:
        origin_angle = angle_between_vectors([0,1], [dirt[0] - current_position.x,dirt[1] - current_position.y])
        true_angle = origin_angle - current_orientation_radian
        if(true_angle < 0): 
            true_angle += 2*math.pi
        if(true_angle > math.pi): 
            true_angle = true_angle - 2*math.pi
        # true_angle = abs(true_angle-2*math.pi) - math.pi
        angles[(dirt[0], dirt[1])] = true_angle

    return angles

def angle_between_vectors(v1, v2):
    return np.math.atan2(np.linalg.det([v1,v2]),np.dot(v1,v2)) + math.pi

def distance_from_line(p1, vec, p0):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p1[0] - vec[0], p1[1] - vec[1]
    x0, y0 = p0[0], p0[1]
    return abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/sqrt((x2-x1)**2+(y2-y1)**2)

def calc_potential(distance): return distance**-2

def update_adversial_position(msg, self):
    last_p = self.opponent_current_p
    self.opponent_current_p = msg.pose.pose.position
    if(last_p is not None):
        opponent_new_vect = Point()
        opponent_new_vect.x = self.opponent_current_p.x - last_p.x
        opponent_new_vect.y = self.opponent_current_p.y - last_p.y
        if(self.opponent_vect is None):
            self.opponent_vect = opponent_new_vect 
        else:
            self.opponent_vect_angle = angle_between_vectors([self.opponent_vect.x,self.opponent_vect.y], [opponent_new_vect.x, opponent_new_vect.y])
            self.opponent_vect = opponent_new_vect 
            self.new_opponent_data = True

class AdvancedCollectorModule(Module):

    def __init__(self, agent_id):
        super(AdvancedCollectorModule, self).__init__(agent_id, 'AdvancedCollectorModule')
        self.cli_cmds = []
        self.finish = False
        self.new_opponent_data = False
        self.opponent_current_p = None
        self.opponent_vect = None
        self.opponent_vect_angle = None
        self.loc_sub = rospy.Subscriber('/tb3_%d/amcl_pose'%self.other_agent_id, PoseWithCovarianceStamped, update_adversial_position, self)

    def update(self): 
        current_p = get_position(self.agent_id)
        # self.print_v("Current location: " + str(current_p.x) + "," + str(current_p.y))

        dirt_list = get_dirt_list()

        if(not dirt_list and not self.finish):
            self.finish = True
            print("no more dirt")
            return 'v'
        elif(not dirt_list):
            return

        self_dirt_distances = get_dirt_distances(self.agent_id, dirt_list)

        if(self.new_opponent_data == True and sqrt(self.opponent_vect.x**2+self.opponent_vect.y**2) > MIN_VECTOR_MAGNITUDE):

            self.new_opponent_data = False 
            
            opponent_dirt_distances = get_dirt_distances(self.other_agent_id, dirt_list, self.agent_id)
            feasible_dirt_list = [dirt for dirt in dirt_list if self_dirt_distances[dirt] < opponent_dirt_distances[dirt]]

            # subsetting the pairs
            opponent_dirt_distances = [(dirt, opponent_dirt_distances[dirt]) for dirt in feasible_dirt_list]
            opponent_dirt_angles = get_dirt_angles(self.other_agent_id, feasible_dirt_list)
            dirt_distance_from_line = list(map(lambda dirt: (dirt, distance_from_line(\
                [self.opponent_current_p.x, self.opponent_current_p.y], [self.opponent_vect.x, self.opponent_vect.y], dirt)), feasible_dirt_list))
            
            potential_dirt_distances = dict(map(lambda pair: (pair[0], calc_potential(pair[1])), opponent_dirt_distances))
            potential_dirt_angles = dict(map(lambda dirt: (dirt, 1/(np.sign(self.opponent_vect_angle)*opponent_dirt_angles[dirt])), opponent_dirt_angles))
            potential_from_line = dict(map(lambda pair: (pair[0], calc_potential(pair[1])), dirt_distance_from_line))

            overall_potential = {}
            for dirt in feasible_dirt_list:
                weight = potential_dirt_distances[dirt]*potential_dirt_angles[dirt]*potential_from_line[dirt]
                overall_potential[dirt] = weight

            # print("\n\n")
            # print("potential_dirt_distances")
            # print(potential_dirt_distances)
            # print("--------------------------------\npotential_dirt_angles")
            # print(potential_dirt_angles)
            # print("--------------------------------\npotential_from_line")
            # print(potential_from_line)
            # print("--------------------------------\noverall_potential")       
            # print(overall_potential)
            # print("\n\n")

            if(overall_potential):
                # min since the calculation was done for the inverse 
                goto_dirt = min(overall_potential, key=overall_potential.get)
                # recall that self_dirt_distances[dirt] < opponent_dirt_distances[dirt]
                p = random.random()
                print("p: " + str(p) +". threshold: " + str((1-self_dirt_distances[dirt] / opponent_dirt_distances[dirt])))
                if((1-self_dirt_distances[dirt] / opponent_dirt_distances[dirt]) > p):
                    self.print_v("mooving to: " + str(goto_dirt) +" instead of: " + str(min(self_dirt_distances, key=self_dirt_distances.get)))
                    movebase_client(self.agent_id, goto_dirt[0], goto_dirt[1], False)
                    return

        goto_dirt = min(self_dirt_distances, key=self_dirt_distances.get)
        movebase_client(self.agent_id, goto_dirt[0], goto_dirt[1])

class BaseCollectorModule(Module):

    def __init__(self, agent_id):
        super(BaseCollectorModule, self).__init__(agent_id, 'BaseCollectorModule')
        self.cli_cmds = []
        self.print_v("Finished init")
        self.finish = False

    def update(self):
        current_p = get_position(self.agent_id)
        # self.print_v("Current location: " + str(current_p.x) + "," + str(current_p.y))

        dirt_list = get_dirt_list()

        if(not dirt_list and not self.finish):
            self.finish = True
            print("no more dirt")
            return 'v'
        elif(not dirt_list):
            return

        updated_dirt_distances = get_dirt_distances(self.agent_id, dirt_list)
        closest_dirt = min(updated_dirt_distances, key=updated_dirt_distances.get)
        movebase_client(self.agent_id, closest_dirt[0], closest_dirt[1]) 

"""
Code for Cleaning:
"""
def vacuum_cleaning(agent_id):
       
    print('start cleaning')
    # robot = Modular([
    #     BaseCollectorModule(agent_id),
    #     # VisitedMapModule(agent_id)
    # ])
    
    if(agent_id == 0):
        robot = Modular([
            BaseCollectorModule(agent_id)
        ])
    elif(agent_id == 1):
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
    """
    The controller that runs the robots, allocate the rooms
    and uses their data the count the shapes
    """
    def __init__(self):
        self.robot0 = Modular([
            RoomInspectorModule(0),
        ])
        self.robot1 = Modular([
            RoomInspectorModule(1),
        ])
        # Initial room allocation:
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
            # Dymanic Area Allocation:
            room_allocation = self.dynamic_room()
            self.robot0.modules[0].set_room(self.room_map == room_allocation[0])
            self.robot1.modules[0].set_room(self.room_map == room_allocation[1])


    def make_rooms(self):
        """ Makes room using KMeans to ensure rooms are as compact
        as possible. This is the methods that calles on initialization (but not while running).
        """
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
        """ Given rooms, send each robot to closest room
        """
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

    # Like make_rooms, but from visited maps instead of the regular map,
    # And base the rooms around the robots
    def dynamic_room(self):
        """ Makes room using KMeans ONLY FROM UNVISITED AREAS.
        This is the methods that called on initialization (but not while running).
        """
        unvisited0 = (self.robot0.modules[0].visited_map == 0)
        unvisited1 = (self.robot1.modules[0].visited_map == 0)
        unvisited = unvisited0 & unvisited1 # Element-wise AND

        points_x, points_y = np.where(unvisited == True)
        points = np.dstack([points_x, points_y])[0, :, :]
        kmeans = KMeans(n_clusters=ROOM_NUMBER, random_state=0).fit(points)
        
        coloring = kmeans.predict(points)
        
        room_map = np.zeros(unvisited.shape)

        for i in range(points.shape[0]):
            room_map[points[i, 0], points[i, 1]] = coloring[i] + 1

        self.room_map = np.fliplr(np.rot90(room_map))
        # tell imshow about color map so that only set colors are used
        # plt.imshow(room_map, cmap=ListedColormap(ROOM_COLORS))
        # plt.show()
        return self.allocate_rooms()

    def count_shapes(self):
        """
        Counting the shapes on the map. This works in 2 steps:
        1. Transfer all the shapes that are large enough from the individual maps
            to a shares combined map.
        2. Count The shapes in the combined map
        """
        CURRENT_COLOR = 25
        PASSED_COLOR = 50
        OBS_COLOR = 100
        # merge the costmaps:
        costmap1 = self.robot0.modules[0].global_costmap
        costmap2 = self.robot1.modules[0].global_costmap

        combined_costmap = np.zeros(costmap1.shape).astype(np.uint8)

        # Pass on each individual map, move the relevent shapes
        # to combined map:
        for i, costmap in enumerate([costmap1, costmap2]):
            thre_map = ((costmap.astype(np.float) / 255) > COSTMAP_THRESHOLD).astype(np.float)
            thre_map = (thre_map*100).astype(np.uint8)
            # First nonempty index:
            x_i, y_i = np.where(thre_map == OBS_COLOR)
            while x_i.shape[0] != 0:
                x = x_i[0]
                y = y_i[0]
                # Fill it
                floodFill(thre_map, None, (y, x), CURRENT_COLOR)
                # if we colored more than PIXEL_THRESHOLD pixels,
                # and the the center of the shape is in the room,
                # add one to the count
                colored_pixels = (thre_map == CURRENT_COLOR).sum()

                if colored_pixels > PIXEL_THRESHOLD:
                    combined_costmap[np.where(thre_map == CURRENT_COLOR)] = OBS_COLOR
                # Return them to a different color
                thre_map[np.where(thre_map == CURRENT_COLOR)] = PASSED_COLOR
                x_i, y_i = np.where(thre_map == OBS_COLOR)
        
        # Now Count in the combined map:
        self.shapes = -1
        x_i, y_i = np.where(combined_costmap == OBS_COLOR)
        while x_i.shape[0] != 0:
            x = x_i[0]
            y = y_i[0]
            # Fill it
            floodFill(combined_costmap, None, (y, x), CURRENT_COLOR)
            # if we colored more than PIXEL_THRESHOLD pixels,
            # and the the center of the shape is in the room,
            # add one to the count
            colored_pixels = (combined_costmap == CURRENT_COLOR).sum()

            if colored_pixels > PIXEL_THRESHOLD:
                self.shapes += 1
            # Return them to a different color
            combined_costmap[np.where(combined_costmap == CURRENT_COLOR)] = PASSED_COLOR
            x_i, y_i = np.where(combined_costmap == OBS_COLOR)
                
        # plt.imshow(combined_costmap, cmap='gray')
        # plt.show()


class RoomInspectorModule(Module):
    """ Module for the the individual robots in the inspection task
    """
    def __init__(self, agent_id):
        super(RoomInspectorModule, self).__init__(agent_id, 'RoomInspectorModule')

        self.cli_cmds = ['v', 'visited_map', 'r', 'room', 'c', 'costmap']
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

        self.loc_sub = rospy.Subscriber(self.get_topic('/move_base/local_costmap/footprint'), PolygonStamped, update_position, self)

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
        if cmd in ['c', 'costmap']:
            plt.imshow(self.global_costmap, cmap='gray')
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