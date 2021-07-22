#!/usr/bin/env python
import rospy
import sys
import numpy as np
from modular import Module
from nav_msgs.msg import OccupancyGrid
from cv2 import floodFill
from matplotlib import pyplot as plt
import matplotlib
# matplotlib.use('Agg')

COSTMAP_THRESHOLD = 0.3
PIXEL_THRESHOLD = 50

class CountModule(Module):

    def __init__(self, agent_id):
        super(CountModule, self).__init__(agent_id, 'CountModule')
        self.cli_cmds = ['c', 'count', 'o', 't']

    def cli(self, cmd):
        if cmd in ['c', 'count']:
            self.update_costmap()
            self.print_v(self.count_shapes())
        elif cmd in ['o']:
            self.update_costmap()
            plt.imshow(self.global_costmap, cmap='gray')
            plt.show()
        elif cmd in ['t']:
            self.update_costmap()
            plt.imshow((self.global_costmap / 255) > COSTMAP_THRESHOLD, cmap='gray')
            plt.show()

    def update(self): 
        pass # todo

    def update_costmap(self):
        map_msg = rospy.wait_for_message(self.get_topic('/move_base/global_costmap/costmap'), OccupancyGrid, timeout=None)
        self.global_costmap = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height))    

    def count_shapes(self):
        map = ((self.global_costmap / 255) > COSTMAP_THRESHOLD).astype(np.float)
        count = -1 # The first one is the outer walls
        # First nonempty index:
        x_i, y_i = np.where(map == 1)
        while x_i.shape[0] != 0:
            x = x_i[0]
            y = y_i[0]
            # Fill it
            map = floodFill(map, (x, y), 0.25)
            # if we colored more than PIXEL_THRESHOLD pixels add one to the count
            colored_pixels = (map == 0.25).sum()
            if colored_pixels > PIXEL_THRESHOLD:
                count += 1
            # Return them to a different color
            map[np.where(map==0.25)] = 0.5
            
            x_i, y_i = np.where(map == 1)
        return count