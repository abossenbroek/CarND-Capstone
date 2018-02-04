import tf
from collections import namedtuple
import math

from car_waypoints import CarPosition, car_coord_waypoints

class VehicleState:
    """
    Implements the vehicle state. It stores velocity, position and dbw status.

    ROS messages can be passed directly and get_state gets the latest information
    """
    def __init__(self):
        self.velocity = {'vel': None, 'theta_dot': None}
        self.position = {'x': None, 'y': None, 'theta': None}
        self.dbw_enabled = None
        self.waypoints = None

    def update_velocity(self, msg):
        self.velocity['vel'] = msg.twist.linear.x
        self.velocity['theta_dot'] = msg.twist.angular.z

    def update_position(self, msg):
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y
        quat = msg.pose.orientation
        quat_array = [quat.x, quat.y, quat.z, quat.w]
        self.position['theta'] = tf.transformations.euler_from_quaternion(quat_array)[2]

    def update_dbw_enabled(self, msg):
        self.dbw_enabled = msg.data

    def update_waypoints(self, msg):
        self.waypoints = msg.waypoints

    def get_state(self):
        return {'vel': self.velocity['vel'],
                'theta_dot': self.velocity['theta_dot'],
                'pos_x': self.position['x'],
                'pos_y': self.position['y'],
                'theta': self.position['theta'],
                'dbw_enabled': self.dbw_enabled}

    def car_coord_waypoints(self):
        cp = CarPosition(self.position['x'], self.position['y'], self.position['theta'])

        return car_coord_waypoints(cp, self.waypoints)
