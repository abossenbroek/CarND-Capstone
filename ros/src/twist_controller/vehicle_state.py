import tf

class VehicleState:
    """
    Implements the vehicle state. It stores velocity, position and dbw status.

    ROS messages can be passed directly and get_state gets the latest information
    """
    def __init__(self):
        self.velocity = [None, None]
        self.position = [None, None, None]
        self.dbw_enabled = None

    def update_velocity(self, msg):
        self.velocity = [msg.twist.linear.x, msg.twist.angular.z]

    def update_position(self, msg):
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        quat = msg.pose.orientation
        quat_array = [quat.x, quat.y, quat.z, quat.w]
        self.position[2] = tf.transformations.euler_from_quaternion(quat_array)[2]

    def update_dbw_enabled(self, msg):
        self.dbw_enabled = msg.data

    def get_state(self):
        return {'vel': self.velocity[0],
                'theta_dot': self.velocity[1],
                'pos_x': self.position[0],
                'pos_y': self.position[1],
                'theta': self.position[2],
                'dbw_enabled': self.dbw_enabled}
