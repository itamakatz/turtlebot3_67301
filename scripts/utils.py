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

def get_position(agent_id):
    try:
        current_location = rospy.wait_for_message('/tb3_%d/amcl_pose'%agent_id, PoseWithCovarianceStamped, 1)
        current_position = current_location.pose.pose.position
    except rospy.ROSException:
        rospy.logerr("waiting for amcl_pose failed.")
        return
    return current_position

def get_dirt_adjacency_mat_distances(agent_id):
    try:
        dirt = rospy.wait_for_message("/dirt",msg.String, 1)
        dirt_list = eval(dirt.data)
    except rospy.ROSException:
        return

    current_position = get_position(agent_id)

    dirt_list.appen([current_position.x, current_position.y])

def get_dirt_list():
    try:
        dirt = rospy.wait_for_message("/dirt",msg.String, 1)
        dirt_list = eval(dirt.data)
        return dirt_list
    except rospy.ROSException:
        rospy.logerr("could not read dirt")
        return []

def get_dirt_distances(agent_id):

    dirt_list = get_dirt_list()

    current_position = get_position(agent_id)

    lengths = {}
    for dirt in dirt_list:

        start = PoseStamped()
        start.header.seq = 0
        start.header.frame_id = "/tb3_%d/map"%agent_id
        start.header.stamp = rospy.Time(0)
        start.pose.position.x = current_position.x
        start.pose.position.y = current_position.y

        Goal = PoseStamped()
        Goal.header.seq = 0
        Goal.header.frame_id = "/tb3_%d/map"%agent_id
        Goal.header.stamp = rospy.Time(0)
        Goal.pose.position.x = dirt[0]
        Goal.pose.position.y = dirt[1]

        req = nav_msgs.srv.GetPlan()
        req.start = start
        req.goal = Goal
        req.tolerance = .5
        # req.tolerance = 2
        
        get_plan = rospy.ServiceProxy('/tb3_%d/move_base/make_plan'%agent_id, nav_msgs.srv.GetPlan)
        resp = get_plan(req.start, req.goal, req.tolerance)
        
        length = poses_to_length(resp.plan.poses)
        lengths[(dirt[0], dirt[1])] = length

    return lengths
    
def poses_to_length(poses_list):
    
    # print("poses len:" + str(len(poses_list)))

    length = 0
    for i in range(len(poses_list) - 1):
        position_a_x = poses_list[i].pose.position.x
        position_b_x = poses_list[i+1].pose.position.x
        position_a_y = poses_list[i].pose.position.y
        position_b_y = poses_list[i+1].pose.position.y

        length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
    return length    

def movebase_client(agent_id,x,y):

    print("moving to: " + str(x) + "," + str(y))

    client = actionlib.SimpleActionClient('/tb3_%d/move_base'%agent_id,MoveBaseAction) # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client.wait_for_server() # Waits until the action server has started up and started listening for goals.
    goal = MoveBaseGoal() # Creates a new goal with the MoveBaseGoal constructor
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0 # No rotation of the mobile base frame w.r.t. map frame
    client.send_goal(goal) # Sends the goal to the action server.
    rospy.loginfo("New goal command received!")
    wait = client.wait_for_result() # Waits for the server to finish performing the action.
   
    if not wait:
        # If the result doesn't arrive, assume the Server is not available
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print("finished!")
        return client.get_result()   # Result of executing the action

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