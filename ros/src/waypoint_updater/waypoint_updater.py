#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number

MAX_DECEL = 5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoint_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        
        self.loop()

        #rospy.spin()

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        
        #rospy.logwarn("closest_idx: {0}".format(closest_idx))

        closest_coord = self.waypoint_2d[closest_idx]
        prev_coord = self.waypoint_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        
        #rospy.logwarn("closest waypoint: {0}".format(cl_vect))
        #rospy.logwarn("position: {0}".format(pos_vect))

        dot_prod = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if dot_prod > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoint_2d)
        return closest_idx

    def publish_waypoints(self):
        lane = Lane()
        #lane.waypoints = self.base_waypoints.waypoints[closest_idx: closest_idx + LOOKAHEAD_WPS]
        final_lane = self.generate_lane()
        #self.final_waypoints_pub.publish(lane)
        self.final_waypoints_pub.publish(final_lane)
        
    def generate_lane(self):
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        
        temp_waypoints = self.base_waypoints.waypoints[closest_idx : farthest_idx]
        
        #rospy.logwarn("Farthest wp id: {0}".format(farthest_idx))
        #rospy.logwarn("Stopline wp id: {0}".format(self.stopline_wp_idx))
        
        
        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx or not(self.stopline_wp_idx):
            lane.waypoints = temp_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(temp_waypoints, closest_idx)
            #rospy.logwarn("Last wp to red light speed: {0}".format(lane.waypoints[LOOKAHEAD_WPS - 1].twist.twist.linear.x))
            
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        stop_idx = max(self.stopline_wp_idx - closest_idx -2, 0)
        
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.0:
                vel = 0.0
            
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            
            temp.append(p)
        
        return temp
       

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoint_2d:
            self.waypoint_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoint_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

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
