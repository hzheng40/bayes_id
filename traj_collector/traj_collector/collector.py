# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped

import numpy as np

class TrajCollector(Node):
    def __init__(self):
        super().__init__('traj_collector')

        # book keeping
        # inputs
        self.steer_input = []
        self.speed_input = []
        # states
        self.x = []
        self.y = []
        self.steer = []
        self.vel_long = []
        self.yaw = []
        self.yawrate = []
        self.slip = []
        # extras
        self.accl_long = []
        self.accl_lat = []
        self.rpm = []
        self.odom_x = []
        self.odom_y = []
        self.odom_yaw = []

        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('pose_topic', '/pf/viz/inferred_pose')
        self.declare_parameter('telemetry_topic', '/sensors/core')
        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.telemetry_topic = self.get_parameter('telemetry_topic').value

        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            self.drive_topic,
            self.drive_callback,
            1)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            1)

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            1)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            1)
        self.telemetry_sub = self.create_subscription(
            VescStateStamped,
            self.telemetry_topic,
            self.telemetry_callback,
            1)

        self.traj_timer = self.create_timer(1.0, self.timer_callback)

    def drive_callback(self, msg):
        self.steer_input.append(msg.drive.steering_angle)
        self.speed_input.append(msg.drive.speed)

    def odom_callback(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)

    def imu_callback(self, msg):
        pass

    def pose_callback(self, msg):
        pass

    def telemetry_callback(self, msg):
        pass

    def timer_callback(self, msg):
        traj = None
        self.all_traj.append(traj)

    def save_traj(self):
        all_traj_np = np.array(self.all_traj)
        np.savez_compressed(self.out_path, all_traj=all_traj_np)


def main(args=None):
    rclpy.init(args=args)
    tc = TrajCollector()
    rclpy.spin(tc)

    # node exit
    tc.save_traj()
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
