#!/usr/bin/env python
import sys
import rospy
import os
import actionlib
from std_msgs import msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from modular import Module
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

from utils import movebase_client, get_dirt_distances, get_position

class BaseCollectorModule(Module):

    def __init__(self, agent_id):
        super(BaseCollectorModule, self).__init__(agent_id, 'BaseCollectorModule')
        self.cli_cmds = []
        # self.dirt_distances = get_dirt_distances(self.agent_id)
        self.print_v("Finished init")

        # import matplotlib.pyplot as plt
        # import numpy as np
        
        # map_msg = rospy.wait_for_message('/tb3_0/map', OccupancyGrid, timeout=None)
        # visited_map = (np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)) != 0) * 100
        # visited_map = np.fliplr(np.rot90(visited_map))

        # map_resolution = map_msg.info.resolution
        # map_origin_translation = map_msg.info.origin.position
        # map_height = visited_map.shape[0]
        # map_width = visited_map.shape[1]
        # from scipy.ndimage.filters import gaussian_filter

        # visited_map = gaussian_filter(visited_map, sigma=7)
        # dirt = rospy.wait_for_message("/dirt",msg.String, 1)
        # dirt_list = np.array(eval(dirt.data))
        # plt.imshow(visited_map, cmap='gray')
        # xs = map(lambda x: map_width - int((x - map_origin_translation.x) / map_resolution), dirt_list[:,1])
        # ys = map(lambda y: map_height - int((y - map_origin_translation.y) / map_resolution) , dirt_list[:,0])
        # plt.scatter(xs, ys, c='w')
        # are_zero = []
        # for k in self.dirt_distances.keys():
        #     if(self.dirt_distances[k] == 0):
        #         x = map_width - int((k[1] - map_origin_translation.x) / map_resolution)
        #         y = map_height - int((k[0] - map_origin_translation.y) / map_resolution)
        #         are_zero.append([x,y])

        # are_zero = np.array(are_zero)
        # if(are_zero):
        #     plt.scatter(are_zero[:,0], are_zero[:,1], c='r')
        # plt.show()

    def update(self): 
        self.print_v("in update")
        current_p = get_position(self.agent_id)
        self.print_v("Current location: " + str(current_p.x) + "," + str(current_p.y))

        # try:
        #     dirt = rospy.wait_for_message("/dirt",msg.String, 1)
        #     dirt_list = eval(dirt.data)
        #     self.print_v("received: " + dirt.data)
        # except rospy.ROSException:
        #     self.print_v("no dirt left")
        #     return

        # if(len(dirt_list) == 0):
        #     self.print_v("no dirt left")
        #     return

        # updated_dirt_distances = {}
        # for dirt in dirt_list:
        #     updated_dirt_distances[(dirt[0], dirt[1])] = self.dirt_distances[(dirt[0], dirt[1])]
        
        updated_dirt_distances = get_dirt_distances(self.agent_id)
        # print(len(updated_dirt_distances))

        if(not updated_dirt_distances):
            print("no more dirt")
            return

        closest_dirt = min(updated_dirt_distances, key=updated_dirt_distances.get)
        # self.print_v(updated_dirt_distances)
        # self.print_v("")
        # self.print_v(closest_dirt)
        # # self.print_v(self.dirt_distances[closest_dirt])
        # self.print_v(updated_dirt_distances[closest_dirt])
        # self.print_v("")
        # self.print_v("next dirt location: " + str(dirt_list[0][0]) + "," + str(dirt_list[0][1]))

        movebase_client(self.agent_id, closest_dirt[0], closest_dirt[1]) 