#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.base_waypoints_2D = None
        self.base_waypoints_tree = None

        # rospy.spin()
        self.loop() # to better control iteration rate

    def loop(self):
    	rate = rospy.Rate(50)
    	while not rospy.is_shutdown():
    		if self.pose and self.base_waypoints:
    			closeset_waypoint_idx = self.get_closest_waypoint_idx()
    			self.publish_waypoints(closeset_waypoint_idx)
    		rate.sleep()

    def get_closest_waypoint_idx(self):
    	cur_x = self.pose.pose.position.x
    	cur_y = self.pose.pose.position.y
    	closest_idx = self.base_waypoints_tree.query([cur_x,cur_y],1)[1]

##################################################################################
    	## Assuming veh is going forward with waypoints increment direction, otherwise car direction info needed to decide closest waypoint
    	closest_coord = self.base_waypoints_2D[closest_idx]
    	prev_coord = self.base_waypoints_2D[(closest_idx-1)%len(base_waypoints_2D)]

    	if np.dot(np.array(closest_coord)-np.array([cur_x,cur_y]), np.array(closest_coord)-np.array(prev_coord))>0:
    		return closest_idx
    	else:
    		return (closest_idx+1)%len(self.base_waypoints_2D)

    def publish_waypoints(self, closest_idx):
    	lane = Lane()
    	lane.header = self.base_waypoints.header
    	lane.waypoints = self.base_waypoints_2D[closest_idx:closest_idx+LOOKAHEAD_WPS]
    	self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.base_waypoints_2D: # initialized before call back
        	self.base_waypoints_2D = [[waypoints.pose.pose.position.x, waypoints.pose.pose.position.y] for waypoint in waypoints.waypoints]
        	self.base_waypoints_tree = KDTree(self.base_waypoints_2D)
        pass

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
