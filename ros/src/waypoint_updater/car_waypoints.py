from collections import namedtuple
import math

CarCoordWayPoint = namedtuple('CarCoordWayPoint', ['x', 'y'])
CarPosition = namedtuple('CarPosition', ['x', 'y', 'theta'])


def car_coord_waypoints(position, waypoints):
    if position.x is None or position.y is None or position.theta is None or len(waypoints) == 0:
        return []

    car_coord_wp = []
    cos_theta = math.cos(position.theta)
    sin_theta = math.sin(position.theta)

    for wp in waypoints:
        dx = wp.pose.pose.position.x - position.x
        dy = wp.pose.pose.position.y - position.y
        cc_wp_x = dx * cos_theta + dy * sin_theta
        cc_wp_y = -dx * sin_theta + dy * cos_theta
        car_coord_wp.append(CarCoordWayPoint(x=cc_wp_x, y=cc_wp_y))

    return car_coord_wp

