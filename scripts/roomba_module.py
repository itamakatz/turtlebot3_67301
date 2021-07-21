import rospy
import sys
import numpy as np
from modular import Module
from matplotlib import pyplot as plt
import matplotlib
from geometry_msgs.msg import Twist
# matplotlib.use('Agg')

SPEED = 0.3
# SPEED = 0.2
ANG_SPEED = 0.4

OPENING_ANGLE_IN_DEGREES = 30

MIN_TURN_SEC = 1
MAX_TURN_SEC = 5

THRESHOLD_DISTANCE = 0.5
DIST_PRINT_THRESHOLD = 0.05

FORWARD_STATE = 0
TURN_STATE = 1

class RoombaModule(Module):
    def __init__(self, agent_id):
        self.cli_cmds = []
        self.base_topic = '/tb3_' + str(agent_id)

        self.vel_pub = rospy.Publisher(self.base_topic + '/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        self.front_min_distance = float('inf')
        self.wide_min_distance = float('inf')
        self.state = FORWARD_STATE

    def update(self):
        if self.state == FORWARD_STATE:
        # Move Forward
            if self.front_min_distance > THRESHOLD_DISTANCE:
                self.vel_msg.linear.x = SPEED
                self.vel_msg.angular.z = 0
                self.vel_pub.publish(self.vel_msg)
            else:
                self.direction = random.uniform(0,1)
                self.state = TURN_STATE
                self.stop()

        elif self.state == TURN_STATE:
            if self.wide_min_distance <= THRESHOLD_DISTANCE:
                self.vel_msg.angular.z = ANG_SPEED if self.direction > 0.5 else -ANG_SPEED
                self.vel_pub.publish(self.vel_msg)     
            else:
                self.state = FORWARD_STATE
                self.stop()


    def stop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_pub.publish(self.vel_msg)

