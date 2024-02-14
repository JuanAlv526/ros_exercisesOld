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


class ComplexPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'fake_scan', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min = -2/3 * math.pi
        msg.angle_max = 2/3 * math.pi
        msg.angle_increment = 1/300 * math.pi
        msg.range_min = 1.0
        msg.range_max = 10.0
        msg.scan_time = 0.1
        msg.time_increment = 0.0

        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [random.uniform(msg.range_min, msg.range_max) for _ in range(num_ranges)]
        msg.intensities = []

        self.publisher_.publish(msg)
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
