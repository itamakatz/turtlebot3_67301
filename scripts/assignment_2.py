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
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn.neighbors import NearestNeighbors
from sklearn.neighbors import kneighbors_graph

from utils import get_position, get_dirt_distances, generate_dirt

ROOM_NUMBER = 2
ROOM_COLORS = ['white', 'red', 'blue']

class AdvancedCollectorModule(Module):

    def __init__(self, agent_id):
        super(AdvancedCollectorModule, self).__init__(agent_id, 'AdvancedCollectorModule')
        self.cli_cmds = []
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

    def update(self): pass

def vacuum_cleaning(agent_id):
       
    print('start cleaning')
    # if(agent_id == 1):
    #     robot = Modular([
    #         AdvancedCollectorModule(agent_id)
    #     ])
    # if(agent_id == 0):
    robot = Modular([
        BaseCollectorModule(agent_id)
    ])
    # else:
    #     raise NotImplementedError

    print("Running modular robot " + str(agent_id))
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
    plt.imshow(room_map, cmap=ListedColormap(ROOM_COLORS))
    plt.show()
    return room_map

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('assignment_2')

    exec_mode = sys.argv[1] 
    print('exec_mode:' + exec_mode)        
    # id = int(sys.argv[2])
    # print_position(id)
    # x = 1.3
    # y = -3.2
    # result = movebase_client(id,x,y)
    # print_position(id)

    # generate_dirt()

    if exec_mode == 'cleaning':        
        agent_id = int(sys.argv[2])
        vacuum_cleaning(agent_id)
        print('agent id:' + sys.argv[2])        
    elif exec_mode == 'inspection':
        inspection()
    else:
        print("Code not found")
        raise NotImplementedError