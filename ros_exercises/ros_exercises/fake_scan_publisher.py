# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import random
import math
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time
from std_msgs.msg import Float32


class ComplexPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')

        self.declare_parameter('fsp_topic', 'fake_scan')
        self.declare_parameter('fsp_rate', 0.05)
        self.declare_parameter('angle_min', -2/3 * math.pi)
        self.declare_parameter('angle_max', 2/3 * math.pi)
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('angle_increment', 1/300 * math.pi)

        # Retrieve parameter values
        self.fsp_topic = self.get_parameter('fsp_topic').value
        self.fsp_rate = self.get_parameter('fsp_rate').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_increment = self.get_parameter('angle_increment').value

        self.publisher_ = self.create_publisher(LaserScan, self.fsp_topic, 10)
        self.range_test_publisher = self.create_publisher(Float32, 'range_test', 10)
        timer_period = self.fsp_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [random.uniform(msg.range_min, msg.range_max) for _ in range(num_ranges)]
        msg.intensities = []

        self.publisher_.publish(msg)

        range_test_msg = Float32()
        range_test_msg.data = float(len(msg.ranges))
        self.range_test_publisher.publish(range_test_msg)

        self.get_logger().info('Publishing LaserScan message')


def main(args=None):
    rclpy.init(args=args)

    fake_scan_publisher = ComplexPublisher()

    rclpy.spin(fake_scan_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_scan_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
