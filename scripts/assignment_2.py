#!/usr/bin/env python
import sys
import rospy
import os
import multi_move_base 
import actionlib
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

from utils import get_position, get_dirt_distances, generate_dirt, movebase_client

COSTMAP_THRESHOLD = 0.25
PIXEL_THRESHOLD = 20

ROOM_NUMBER = 2
ROOM_COLORS = ['white', 'red', 'blue']

class AdvancedCollectorModule(Module):

    def __init__(self, agent_id):
        super(AdvancedCollectorModule, self).__init__(agent_id, 'AdvancedCollectorModule')
        self.cli_cmds = []
        self.finish = False
        dirt = rospy.wait_for_message("/dirt",msg.String, 1)
        dirt_list = np.array(eval(dirt.data))
        self.print_v(dirt_list,True)
        # X = np.array([[-1, -1], [-2, -1], [-3, -2], [1, 1], [2, 1], [3, 2]])
        X = np.array([[-1, -1], [3, 2]])
        nbrs = NearestNeighbors(n_neighbors=len(dirt_list), algorithm='ball_tree').fit(dirt_list)
        distances, indices = nbrs.kneighbors(X)
        self.print_v(indices,True)
        self.print_v(distances,True)
        self.print_v(nbrs.kneighbors_graph(X).toarray(),True)
        
        # plt.scatter(dirt_list[:,0], dirt_list[:,1], c='#00FF00', label="dirt_list")
        # plt.scatter(X[:,0], X[:,1], c='#FFFF00', label="X")
        # plt.show()

        lengths1 = get_dirt_distances(self.agent_id)
        lengths1
        self.print_v(lengths1)
        lengths2 = get_dirt_distances(self.get_other_id())
        self.print_v(lengths2)

    def update(self): 
        current_p = get_position(self.agent_id)
        self.print_v("Current location: " + str(current_p.x) + "," + str(current_p.y))
        updated_dirt_distances = get_dirt_distances(self.agent_id)

        if(not updated_dirt_distances and not self.finish):
            self.finish = True
            print("no more dirt")
            return 'v'

        closest_dirt = min(updated_dirt_distances, key=updated_dirt_distances.get)
        movebase_client(self.agent_id, closest_dirt[0], closest_dirt[1])         

def vacuum_cleaning(agent_id):
       
    print('start cleaning')
    robot = Modular([
        BaseCollectorModule(agent_id),
        VisitedMapModule(agent_id)
    ])
    
    # if(agent_id == 1):
    #     robot = Modular([
    #         BaseCollectorModule(agent_id)

    #     ])
    # if(agent_id == 0):
    #     robot = Modular([
    #         # AdvancedCollectorModule(agent_id)
    #         BaseCollectorModule(agent_id)
    #     ])
    # else:
    #     raise NotImplementedError

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
    # merge the costmaps:
    costmap = np.maximum(costmap1, costmap2)
    thre_map = ((costmap / 255) > COSTMAP_THRESHOLD).astype(np.float)
    # plt.imshow(costmap, cmap='gray')
    # plt.show()
    # plt.imshow(thre_map, cmap='gray')
    # plt.show()
    count = -2 # The first one is the outer walls
    # First nonempty index:
    x_i, y_i = np.where(thre_map == 1)
    while x_i.shape[0] != 0:
        x = x_i[0]
        y = y_i[0]
        # Fill it
        thre_map = floodFill(thre_map, (x, y), 0.25)
        # if we colored more than PIXEL_THRESHOLD pixels add one to the count
        colored_pixels = (thre_map == 0.25).sum()
        if colored_pixels > PIXEL_THRESHOLD:
            count += 1
        # Return them to a different color
        thre_map[np.where(thre_map==0.25)] = 0.5
        
        x_i, y_i = np.where(thre_map == 1)
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