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
from ros_exercises_interfaces.msg import OpenSpace



class ComplexSubscriber(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.declare_parameter('osp_sub_topic', 'fake_scan')
        self.declare_parameter('osp_pub_topic', 'open_space')

        self.osp_sub_topic = self.get_parameter('osp_sub_topic').value
        self.osp_pub_topic = self.get_parameter('osp_pub_topic').value


        self.subscription = self.create_subscription(
            LaserScan,
            self.osp_sub_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.open_publisher_ = self.create_publisher(OpenSpace, self.osp_pub_topic, 10)
      

    def listener_callback(self, msg):
        longest_range = max(msg.ranges)
        index_of_longest_range = msg.ranges.index(longest_range)
        longest_range_angle = msg.angle_min + index_of_longest_range * msg.angle_increment
        self.get_logger().info('Longest range: %f at angle: %f' % (longest_range, longest_range_angle))

        open_msg = OpenSpace()
        open_msg.angle = longest_range_angle
        open_msg.distance = longest_range

        
        self.open_publisher_.publish(open_msg)
       


def main(args=None):
    rclpy.init(args=args)
    open_space_publisher = ComplexSubscriber()
    rclpy.spin(open_space_publisher)
    open_space_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
