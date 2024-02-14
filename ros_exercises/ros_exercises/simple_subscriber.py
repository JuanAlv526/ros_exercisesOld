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
import math

from std_msgs.msg import Float32


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'my_random_float',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Float32, 'random_float_log', 10)

    def listener_callback(self, msg):
        log_value = math.log(msg.data)  # Calculate natural log
        self.get_logger().info('Received: %f, Log: %f' % (msg.data, log_value))
        log_msg = Float32()
        log_msg.data = log_value
        self.publisher_.publish(log_msg)


def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = MinimalSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
