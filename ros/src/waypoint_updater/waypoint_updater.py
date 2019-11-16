#!/usr/bin/env python

import math
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial import KDTree

from styx_msgs.msg import Lane

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_wp = None
        self.base_wp_2d = None
        self.base_wp_tree = None
        self.current_pose = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.current_pose and self.base_wp_tree:
                idx = self.find_closest_waypoint_idx()
                self.publish_waypoints(idx)
                rate.sleep()

    def find_closest_waypoint_idx(self):
        pos = np.array([self.current_pose.x, self.current_pose.y])
        closest_idx = self.base_wp_tree.query(pos)[1]
        closest = self.base_wp_2d[closest_idx]

        direction = np.dot(closest - self.base_wp_2d[closest_idx - 1], pos - closest)
        if direction > 0:
            closest_idx = (closest_idx + 1) % len(self.base_wp_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_wp.header
        lane.waypoints = self.base_wp.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        lane.waypoints = lane.waypoints + self.base_wp.waypoints[:LOOKAHEAD_WPS - len(lane.waypoints)]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.current_pose = msg.pose.position

    def waypoints_cb(self, waypoints):
        if not self.base_wp:
            self.base_wp = waypoints
            self.base_wp_2d = np.array(
                [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in waypoints.waypoints])
            self.base_wp_tree = KDTree(self.base_wp_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
