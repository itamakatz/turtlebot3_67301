import random
import actionlib
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs import msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import nav_msgs

def generate_dirt():
    map_msg = rospy.wait_for_message('/tb3_0/map', OccupancyGrid, timeout=None)
    visited_map = (np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)) != 0) * 100
    visited_map = np.fliplr(np.rot90(visited_map))

    map_resolution = map_msg.info.resolution
    map_origin_translation = map_msg.info.origin.position
    map_height = visited_map.shape[0]
    map_width = visited_map.shape[1]
    from scipy.ndimage.filters import gaussian_filter

    visited_map = gaussian_filter(visited_map, sigma=7)

    indices = np.where(visited_map == 0)
    sampled_list = random.sample(range(len(indices[0])), 50)
    plt.imshow(visited_map, cmap='gray')
    plt.scatter(indices[1][sampled_list], indices[0][sampled_list], c='w')
    plt.show()
    xs = map(lambda x: round(((map_height - x) * map_resolution) + map_origin_translation.x ,2) ,indices[1][sampled_list])
    ys = map(lambda y: round(((map_width  - y) * map_resolution) + map_origin_translation.y ,2) ,indices[0][sampled_list])
    print(list(zip(ys, xs)))

def print_position(agent_id):
    mMap_mdata = rospy.wait_for_message('/tb3_%d/map_metadata'%agent_id,MapMetaData)
    location = rospy.wait_for_message('/tb3_%d/amcl_pose'%agent_id, PoseWithCovarianceStamped)
    p = location.pose.pose.position
    x_id = int((p.x - mMap_mdata.origin.position.x)/mMap_mdata.resolution)
    y_id = int((p.y - mMap_mdata.origin.position.y)/mMap_mdata.resolution)
    print("Current location: " + str(p.x) + "," + str(p.y))
    print("Current location in the map: " + str(x_id) + "," + str(y_id))