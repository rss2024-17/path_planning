import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage #https://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html

from .utils import LineTrajectory

import numpy as np


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lookahead = 4  # FILL IN #
        self.speed = 1.0  # FILL IN #
        self.wheelbase_length = 0  # FILL IN #

        self.trajectory = LineTrajectory("/followed_trajectory")
        self.initialized_traj = False

        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic,
                                                 self.pose_callback,
                                                 1)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # self.clicked_point = (0, 0)
        # self.click_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.arrow_callback, 10)

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.pose_callback,
                                                 1)
        
        # self.tf_pub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        # self.sim_pos_x = 0.0
        # self.sim_pos_y = 0.0
        # self.sim_theta = 0.0

        # from particle filter 
        # self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/base_link_pf', self.pose_callback, 10)

    # TEMP
    # def arrow_callback(self, msg_click):
    #     self.x = msg_click.pose.pose.point.x
    #     self.y = msg_click.point.y
    #     self.get_logger().info("Point clicked at %f %f" % (x, y))

    #     point = self.find_next_point()
    #     # self.generate_random_particles(x,y)
    #     # self.update_particles_viz()

    def pose_callback(self, update_pose):
        # https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseWithCovarianceStamped.html

        position = update_pose.pose.pose.position
        orientation = update_pose.pose.pose.orientation
        
        self.x = position.x
        self.y = position.y

        orientation = quaternion_to_euler(orientation.w, orientation.x, orientation.y, orientation.z)

        self.yaw = orientation[2] # we just want the yaw

        self.get_logger().info("Set pose to x: %f y: %f theta: %f" % (self.x, self.y, self.yaw))

        # for later when we switch to odometry
        # self.x = odometry_msg.pose.pose.position.x
        # self.y = odometry_msg.pose.pose.position.y
        # self.yaw = quaternion_to_euler(odometry_msg.pose.pose.orientation.w, odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z)[2]

        point = self.find_next_point()


    def intersection_point(self, segment, center_circle):
        # https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm/86428#86428 

        # formatting: segment [[-52.59911875073924, 0.3921607738265719], [-52.69975808361432, 0.4931211855300348]]
        v = [segment[1][0] - segment[0][0], segment[1][1] - segment[0][1]]
        self.get_logger().info(f"in interesection point")
        
        a = np.dot(v, v)
        b = 2 * (np.dot(v, [segment[0][0] - center_circle[0], segment[0][1] - center_circle[1]]))
        c = np.dot(segment[0], segment[0]) + np.dot(center_circle, center_circle) - 2 * np.dot(segment[0], center_circle) - self.lookahead**2
        
        disc = b**2 - 4 * a * c

        # if not real discriminant
        if disc < 0:
            return None
        
        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)

        # if t1 or t2 are not on the segment
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return None

        return t1, t2

        # they did
        # t = max(0, min(1, - b / (2 * a)))
        # return True, segment[0] + t * v 
        
    # in robot frame
    def find_next_point(self):
        points = self.trajectory.points
        num_points = len(points)

        min_dist = self.min_dist_squared(points[0], points[1], [self.x, self.y])
        min_dist_i = 0
        for i in range(num_points-1):
            # compute distance to segment between points i and i+1
            dist = self.min_dist_squared(points[i], points[i+1], [self.x, self.y])
            if (dist < min_dist):
                min_dist = dist 
                min_dist_i = i
        
        self.get_logger().info("Closest segment found:")
        self.get_logger().info(f"From {points[min_dist_i][0]} {points[min_dist_i][1]} to {points[min_dist_i+1][0]} {points[min_dist_i+1][1]}")
        self.get_logger().info("Now searching for line-circle intersection...")

        # min_dist_i is the index of the closest segment to the robot 
        point = None
        max_ahead = 5   # can only look this many segments ahead from min_dist_i
        ahead = 0

        center_circle = [self.x, self.y]
        
        # while we haven't found a valid point,
        # we haven't looked ahead more than max_ahead segments,
        # and we haven't reached the final point in the trajectory:
        while(point == None and ahead < max_ahead and ahead + min_dist_i < num_points-1):
            segment = [
                [points[min_dist_i+ahead][0], points[min_dist_i+ahead][1]],
                [points[min_dist_i+ahead+1][0], points[min_dist_i+ahead+1][1]]
            ]
            # self.get_logger().info(f"segment {segment}")
            point = self.intersection_point(segment, center_circle)
            ahead += 1
        
        if (point == None): # no point was found lol
            self.get_logger().info("no point was found hahahahahahhaa")
            return None
    
        self.get_logger().info(f"{point[0]} {point[1]}")
        
        return point

    # returns the min dist squared between point and the segment with endpoints start and end
    def min_dist_squared(self, start, end, point):
        dist_squared = self.dist_squared(start, end)

        if (dist_squared == 0): return self.dist_squared(start, point)

        t = max(0, min(1, self.dot_product([point[0] - start[0], point[1] - start[1]], [end[0] - start[0], end[1] - start[1]]) / dist_squared))
        # projection = start + t * (end - start) # Projection falls on the segment
        return t

    def dot_product(self, p1, p2):
        return p1[0] * p2[0] + p1[1] * p2[1]
    
    def dist_squared(self, p1, p2):
        return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 
    
    def world_to_robot_frame(self, world_pose, robot_pose):
        theta = robot_pose[2]
        next_pose = robot_pose[0:2] + np.array([np.cos(theta) * world_pose[0] + -np.sin(theta) * world_pose[1], 
                                                np.sin(theta) * world_pose[0] + np.cos(theta) * world_pose[1]])
        next_theta = (theta + world_pose[2]) % (2 * 3.14159)

        return np.array([next_pose[0], next_pose[1], next_theta])
    
    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.get_logger().info("printing guys")
        # self.get_logger().info(len(self.trajectory.points))
        for point in self.trajectory.points:
            self.get_logger().info(f"{point[0]} {point[1]}")

        self.initialized_traj = True

    # def tf_callback(self, msg_tf):
    #     frame = msg_tf.transforms[0].header.frame_id
    #     child = msg_tf.transforms[0].child_frame_id
    #     if frame == 'map' and child == 'base_link':
    #         self.sim_pos_x = msg_tf.transforms[0].transform.translation.x
    #         self.sim_pos_y = msg_tf.transforms[0].transform.translation.y
    #         self.sim_theta = quaternion_to_euler(msg_tf.transforms[0].transform.rotation.w, msg_tf.transforms[0].transform.rotation.x, msg_tf.transforms[0].transform.rotation.y, msg_tf.transforms[0].transform.rotation.z)[2]



def quaternion_to_euler(w, x, y, z):

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = np.sqrt(1 + 2 * (w * y - x * z))
    cosp = np.sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()



    