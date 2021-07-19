import rospy
import sys
import numpy as np
from modular import Module
from nav_msgs.msg import OccupancyGrid 
from geometry_msgs.msg import PolygonStamped
from matplotlib import pyplot as plt
import matplotlib
# matplotlib.use('Agg')

RADIUS = 5
VISITED_COLOR = 50

class VisitedMapModule(Module):
    def __init__(self, agent_id):
        self.cli_cmds = ['v', 'visited_map']
        self.base_topic = '/tb3_' + str(agent_id)
        self.loc_sub = rospy.Subscriber(self.base_topic + '/move_base/local_costmap/footprint', PolygonStamped, update_position, self)

        print("Setting up visited map module...")
        map_msg = rospy.wait_for_message(self.base_topic+'/map', OccupancyGrid, timeout=None)
        self.visited_map = (np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)) != 0) * 100
        self.visited_map = np.fliplr(np.rot90(self.visited_map))
        self.map_height = self.visited_map.shape[0] # NOTE - this may be revered, as in map_height=shape[1] etc.
        self.map_width = self.visited_map.shape[1]
        self.starting_unexplored = (self.visited_map == 0).sum()
        self.percent_array = np.zeros((1,2))
        self.start_time = rospy.get_rostime().secs
        print("Finished setting up visited map module")


    def cli(self, cmd):
        plt.imshow(self.visited_map, cmap='gray')
        plt.show()

    def update(self):
        print('u')


    def footprint_to_map(self, x, y):
        map_y = self.map_width - int((x + 10) * 20)
        map_x = self.map_height - int((y + 10) * 20)
        return (map_x, map_y)

    def map_to_footprint(self, x, y):
        footprint_y = ((self.map_height - x) / 20) - 10
        footprint_x = ((self.map_width - y) / 20) - 10
        return (footprint_x, footprint_y)


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