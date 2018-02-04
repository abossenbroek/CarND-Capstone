#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from car_waypoints import CarPosition, car_coord_waypoints

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

        self.current_pose = None
        self.all_waypoints = None
        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg

        if self.all_waypoints is None:
            return

        quat = msg.pose.orientation
        quat_array = [quat.x, quat.y, quat.z, quat.w]
        theta = tf.transformations.euler_from_quaternion(quat_array)[2]

        # Now that we have an update on position we can determine which waypoints are ahead of us
        cp = CarPosition(msg.pose.position.x, msg.pose.position.y, theta)
        rel_wp = car_coord_waypoints(cp, self.all_waypoints)
        wps = [[rel, org] for rel, org in zip(rel_wp, self.all_waypoints)]
        sorted_wps = sorted(wps, key=lambda wp: math.sqrt((wp[0].x - cp.x)**2 + (wp[0].y - cp.y)**2))
        orig_waypoints = [wp[1] for wp in sorted_wps]


        final_lane = Lane()
        final_lane.header.frame_id = '/final_waypoints'
        final_lane.header.stamp = rospy.Time(0)
        final_lane.waypoints = orig_waypoints[:LOOKAHEAD_WPS]
        #for i, wp in enumerate(final_lane.waypoints):
            #rospy.loginfo("for waypoint {} found x: {}, y: {}".format(i, wp.pose.pose.position.x, wp.pose.pose.position.y))

        self.final_waypoints_pub.publish(final_lane)

    def waypoints_cb(self, lane):
        # TODO: Implement
        # TODO: determine what the waypoints will be, will serve as input to DBW
        # Store the waypoints for future reference
        if self.all_waypoints is None:
            self.all_waypoints = lane.waypoints

        assert self.current_pose is None

        final_lane = Lane()
        final_lane.header.frame_id = '/final_waypoints'
        final_lane.header.stamp = rospy.Time(0)
        # we don't have a current position so we just push the list of waypoints
        # we received directly through to the list that gets pushed to final_waypoints
        final_lane.waypoints = lane.waypoints[:LOOKAHEAD_WPS]
        #for i, wp in enumerate(final_lane.waypoints):
            #rospy.loginfo("for waypoint {} found x: {}, y: {}".format(i, wp.pose.pose.position.x, wp.pose.pose.position.y))

        self.final_waypoints_pub.publish(final_lane)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # TODO: take action once a traffic light with certain red state is observed
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # TODO: consider moving around waypoint using path planning
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
