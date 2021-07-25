#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
from modular import Module

class LocationModule(Module):

    def __init__(self, agent_id):
        super(LocationModule, self).__init__(agent_id, 'LocationModule')
        self.cli_cmds = ['location']

    def update(self):
        mMap_mdata = rospy.wait_for_message(self.get_topic('/map_metadata'), MapMetaData, 1)
        location = rospy.wait_for_message(self.get_topic('/amcl_pose'), PoseWithCovarianceStamped, 1)
        p = location.pose.pose.position
        x_id = int((p.x - mMap_mdata.origin.position.x)/mMap_mdata.resolution)
        y_id = int((p.y - mMap_mdata.origin.position.y)/mMap_mdata.resolution)
        self.print_v("Current location: " + str(p.x) + "," + str(p.y))
        self.print_v("Current location in the map: " + str(x_id) + "," + str(y_id))
        return None