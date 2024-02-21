import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np


class StaticTFCamPublisher(Node):

    def __init__(self):
        super().__init__('static_tf_cam_publisher')

        self.br = tf2_ros.StaticTransformBroadcaster(self)

        self.left_cam_offset = np.array([0.05, 0.0, 0.0])
        self.right_cam_offset = np.array([-0.05, 0.0, 0.0])

        self.publish_transforms()

    def publish_transforms(self):
        # Static transform from base_link_gt to left_cam
        left_transform = TransformStamped()
        left_transform.header.stamp = self.get_clock().now().to_msg()
        left_transform.header.frame_id = 'base_link_gt'
        left_transform.child_frame_id = 'left_cam'
        left_transform.transform.translation.x = self.left_cam_offset[0]
        left_transform.transform.translation.y = self.left_cam_offset[1]
        left_transform.transform.translation.z = self.left_cam_offset[2]
        left_transform.transform.rotation.w = 1.0  # Identity quaternion
        self.br.sendTransform(left_transform)

        # Static transform from base_link_gt to right_cam
        right_transform = TransformStamped()
        right_transform.header.stamp = self.get_clock().now().to_msg()
        right_transform.header.frame_id = 'base_link_gt'
        right_transform.child_frame_id = 'right_cam'
        right_transform.transform.translation.x = self.right_cam_offset[0]
        right_transform.transform.translation.y = self.right_cam_offset[1]
        right_transform.transform.translation.z = self.right_cam_offset[2]
        right_transform.transform.rotation.w = 1.0  # Identity quaternion
        self.br.sendTransform(right_transform)


def main(args=None):
    rclpy.init(args=args)
    static_tf_cam_publisher = StaticTFCamPublisher()
    rclpy.spin(static_tf_cam_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
