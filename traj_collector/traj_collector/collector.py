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
import tf_transformations

class TrajCollector(Node):
    """
    With single track model, need 7 states:
    [x, y, steer, long_v, yaw, yawrate, slip_angle]
    """
    def __init__(self):
        super().__init__('traj_collector')

        # book keeping
        self.all_traj = []
        self.all_input = []
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

        # timekeeping
        self.last_save_time = None

        # 20 hz
        self.declare_parameter('drive_topic', '/ackermann_cmd')
        # 50 hz
        self.declare_parameter('odom_topic', '/odom')
        # 50 hz
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        # 25 hz
        self.declare_parameter('pf_odom_topic', '/pf/pose/odom')
        self.declare_parameter('telemetry_topic', '/sensors/core')
        self.declare_parameter('output_path', '/home/billyzheng/bags/saved_traj.npz')
        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pf_odom_topic = self.get_parameter('pf_odom_topic').value
        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.out_path = self.get_parameter('output_path').value

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
        self.pf_odom_sub = self.create_subscription(
            Odometry,
            self.pf_odom_topic,
            self.pf_odom_callback,
            1)
        self.telemetry_sub = self.create_subscription(
            VescStateStamped,
            self.telemetry_topic,
            self.telemetry_callback,
            1)

        self.traj_timer = self.create_timer(1.0, self.timer_callback)

    def drive_callback(self, msg):
        self.steer_input.append(msg.drive.steering_angle)
        self.steer.append(msg.drive.steering_angle)
        self.speed_input.append(msg.drive.speed)

    def odom_callback(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)

    def imu_callback(self, msg):
        pass

    def pf_odom_callback(self, msg):
        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        self.vel_long.append(msg.twist.twist.linear.x)
        self.yaw.append(tf_transformations.euler_from_quaternion(q)[2])

    def telemetry_callback(self, msg):
        pass

    def reset_states(self):
        self.steer_input = []
        self.speed_input = []
        self.x = []
        self.y = []
        self.steer = []
        self.vel_long = []
        self.yaw = []
        self.yawrate = []
        self.slip = []
        self.accl_long = []
        self.accl_lat = []
        self.rpm = []
        self.odom_x = []
        self.odom_y = []
        self.odom_yaw = []

    def timer_callback(self):
        # yawrate
        num_yaw = len(self.yaw)
        self.yaw = np.array(self.yaw)
        dt_yaw = 1. / num_yaw
        yaw_diff = self.yaw[1:] - self.yaw[:-1]
        self.yawrate = yaw_diff / dt_yaw

        # vectors between x, y
        self.x = np.array(self.x)
        self.y = np.array(self.y)
        x_diff = self.x[1:] - self.x[:-1]
        y_diff = self.y[1:] - self.y[:-1]
        v_ang = np.arctan2(y_diff, x_diff)

        # slip angle
        if len(v_ang) > len(self.yaw):
            v_ang = v_ang[:len(self.yaw)]
        elif len(v_ang) == len(self.yaw):
            pass
        else:
            self.yaw = self.yaw[:len(v_ang)]

        self.slip = v_ang - self.yaw

        # subsample to length 10 for each second
        # steer, speed input, steer input all has same length
        len_input = len(self.steer)
        skip_input = int(len_input / 10.)
        if skip_input <= 1:
            self.reset_states()
            return
        self.steer = np.array(self.steer)
        self.steer_input = np.array(self.steer_input)
        self.speed_input = np.array(self.speed_input)
        self.steer = self.steer[::skip_input]
        self.steer_input = self.steer_input[::skip_input]
        self.speed_input = self.speed_input[::skip_input]

        # from pf odom
        len_pos = len(self.x)
        skip_pos = int(len_pos / 10.)
        if skip_pos <= 1:
            self.reset_states()
            return
        self.x = self.x[::skip_pos]
        self.y = self.y[::skip_pos]
        self.vel_long = np.array(self.vel_long)[::skip_pos-1]
        self.yaw = self.yaw[::skip_pos]
        self.slip = self.slip[::skip_pos]
        self.yawrate = self.yawrate[::skip_pos]

        min_len = min(10, len(self.x), len(self.y), len(self.steer), len(self.vel_long), len(self.yaw), len(self.yawrate), len(self.slip))
        if min_len < 10:
            self.reset_states()
            return
        self.x = self.x[:min_len]
        self.y = self.y[:min_len]
        self.steer = self.steer[:min_len]
        self.vel_long = self.vel_long[:min_len]
        self.yaw = self.yaw[:min_len]
        self.yawrate = self.yawrate[:min_len]
        self.slip = self.slip[:min_len]
        self.steer_input = self.steer_input[:min_len]
        self.speed_input = self.speed_input[:min_len]
        traj = np.stack((self.x, self.y, self.steer, self.vel_long, self.yaw, self.yawrate, self.slip))
        traj_input = np.stack((self.steer_input, self.speed_input))
        self.all_traj.append(traj)
        self.all_input.append(traj_input)

        # reset
        self.reset_states()

        self.save_traj()

    def save_traj(self):
        all_traj_np = np.array(self.all_traj)
        all_input_np = np.array(self.all_input)
        np.savez_compressed(self.out_path, traj=all_traj_np, input=all_input_np)


def main(args=None):
    rclpy.init(args=args)
    tc = TrajCollector()
    rclpy.spin(tc)

    # node exit
    # tc.save_traj()
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
