#!/usr/bin/env python
import sys
import rospy
import os
import multi_move_base 
import actionlib
from math import sqrt
from visited_map_module import VisitedMapModule
from room_inspector_module import RoomInspectorModule
from std_msgs import msg
from roomba_module import RoombaModule
from nav_msgs.msg import OccupancyGrid, MapMetaData
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from modular import Module, Modular
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

COSTMAP_THRESHOLD = 0.25
PIXEL_THRESHOLD = 20

ROOM_NUMBER = 2
ROOM_COLORS = ['white', 'red', 'blue']

MAX_DIRT_DISTANCE = 2
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

def inspection():
    room_map = make_rooms()
    print('start inspection')
    robot1 = Modular([
        RoomInspectorModule(0, room_map == 1),
    ])
    robot2 = Modular([
        RoomInspectorModule(1, room_map == 2),
    ])
    print("Running modular robots ")
    while(not rospy.is_shutdown()):
        robot1.run_single_round()
        robot2.run_single_round()
        shapes = count_shapes(robot1.modules[0].global_costmap, robot2.modules[0].global_costmap)
        print("Number of shapes: ", shapes)

def make_rooms():
    map_msg = rospy.wait_for_message('/tb3_0/map', OccupancyGrid, timeout=None)
    map = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)) != 0
    points_x, points_y = np.where(map == False)
    points = np.dstack([points_x, points_y])[0, :, :]
    kmeans = KMeans(n_clusters=ROOM_NUMBER, random_state=0).fit(points)
    
    coloring = kmeans.predict(points)
    
    room_map = np.zeros(map.shape)

    for i in range(points.shape[0]):
        room_map[points[i, 0], points[i, 1]] = coloring[i] + 1

    room_map = np.fliplr(np.rot90(room_map))
    # tell imshow about color map so that only set colors are used
    # plt.imshow(room_map, cmap=ListedColormap(ROOM_COLORS))
    # plt.show()
    return room_map

def count_shapes(costmap1, costmap2):
    CURRENT_COLOR = 25
    PASSED_COLOR = 50
    OBS_COLOR = 100
    # merge the costmaps:
    costmap = np.maximum(costmap1, costmap2)
    # print("costmap max is ", np.max(costmap))
    # print("costmap min is ", np.min(costmap))
    thre_map = ((costmap.astype(np.float) / 255) > COSTMAP_THRESHOLD).astype(np.float)
    thre_map = (thre_map*100).astype(np.uint8)
    # print("threshold max is ", np.max(thre_map))
    # print("threshold min is ", np.min(thre_map))
    # plt.imshow(costmap, cmap='gray')
    # plt.show()
    # plt.imshow(thre_map, cmap='gray')
    # plt.show()
    count = -3 # The first one is the outer walls, two robots
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
    return count


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