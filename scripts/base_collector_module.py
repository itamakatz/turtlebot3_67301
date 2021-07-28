#!/usr/bin/env python
import os
import sys
import rospy
import actionlib
from modular import Module
from utils import movebase_client, get_dirt_distances, get_position

class BaseCollectorModule(Module):

    def __init__(self, agent_id):
        super(BaseCollectorModule, self).__init__(agent_id, 'BaseCollectorModule')
        self.cli_cmds = []
        self.print_v("Finished init")
        self.finish = False

    def update(self): 
        current_p = get_position(self.agent_id)
        self.print_v("Current location: " + str(current_p.x) + "," + str(current_p.y))
        updated_dirt_distances = get_dirt_distances(self.agent_id)

        if(not updated_dirt_distances and not self.finish):
            if()
            self.finish = True
            print("no more dirt")
            return 'v'
        elif(not updated_dirt_distances):
            return
            
        closest_dirt = min(updated_dirt_distances, key=updated_dirt_distances.get)
        movebase_client(self.agent_id, closest_dirt[0], closest_dirt[1]) 