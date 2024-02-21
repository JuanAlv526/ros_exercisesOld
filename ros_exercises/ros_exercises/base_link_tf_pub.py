import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_transformations as tft

class BaseLinkTFPublisher(Node):

    def __init__(self):
        super().__init__('base_link_tf_pub')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.br = tf2_ros.TransformBroadcaster(self)

        # Timer to periodically publish transform
        timer_period = 1.0 / 20  # 20 Hz
        self.timer = self.create_timer(timer_period, self.publish_base_link_tf)

    def publish_base_link_tf(self):
        try:
            # Lookup transform from left_cam to world
            tf_left_cam_to_world = self.tfBuffer.lookup_transform('world', 'left_cam', rclpy.time.Time())
            
            # Lookup transform from base_link to left_cam
            tf_base_link_to_left_cam = self.tfBuffer.lookup_transform('left_cam', 'base_link_gt', rclpy.time.Time())

            # Convert transform messages to numpy arrays
            left_cam_to_world_matrix = self.tf_to_matrix(tf_left_cam_to_world.transform)
            base_link_to_left_cam_matrix = self.tf_to_matrix(tf_base_link_to_left_cam.transform)

            # Compute transform from world to base_link_gt_2
            world_to_base_link_gt_2_matrix = np.dot(left_cam_to_world_matrix, np.linalg.inv(base_link_to_left_cam_matrix))

            # Publish the transform
            self.publish_tf(world_to_base_link_gt_2_matrix, 'world', 'base_link_gt_2')

        except tf2_ros.LookupException as e:
            self.get_logger().info(f'Failed to lookup transform: {e}')

    def tf_to_matrix(self, transform):
        q = transform.rotation
        q = [q.x, q.y, q.z, q.w]
        t = transform.translation
        mat = tft.quaternion_matrix(q)
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z
        return mat

    def publish_tf(self, transform, parent_frame, child_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = transform[0, 3]
        t.transform.translation.y = transform[1, 3]
        t.transform.translation.z = transform[2, 3]
        quaternion = tft.quaternion_from_matrix(transform)
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    base_link_tf_pub = BaseLinkTFPublisher()
    rclpy.spin(base_link_tf_pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
