#===========================================================================================
#Try 2
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import geometry_msgs
import tf_transformations
import time

from geometry_msgs.msg import TransformStamped

from typing import Any


class DynamicTFCamPublisher(Node):

    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # broadcaster that will publish the transform
        self.br = tf2_ros.TransformBroadcaster(self)

        timer_period = 1.0 / 20  # hz
        self.timer = self.create_timer(timer_period, self.node_callback)

        self.left_cam_offset = np.array([0.05, 0.0, 0.0])
        self.right_cam_offset = np.array([-0.05, 0.0, 0.0])


    def tf_to_se3(self, transform: TransformStamped.transform) -> np.ndarray:
            """
            Convert a TransformStamped message to a 4x4 SE3 matrix 
            """
            q = transform.rotation
            q = [q.x, q.y, q.z, q.w]
            t = transform.translation
            mat = tf_transformations.quaternion_matrix(q)
            mat[0, 3] = t.x
            mat[1, 3] = t.y
            mat[2, 3] = t.z
            return mat

    def se3_to_tf(self, mat: np.ndarray, time: Any, parent: str, child: str) -> TransformStamped:
        """
        Convert a 4x4 SE3 matrix to a TransformStamped message
        """
        obj = geometry_msgs.msg.TransformStamped()

        # current time
        obj.header.stamp = self.get_clock().now().to_msg()

        # frame names
        obj.header.frame_id = parent
        obj.child_frame_id = child

        # translation component
        obj.transform.translation.x = mat[0, 3]
        obj.transform.translation.y = mat[1, 3]
        obj.transform.translation.z = mat[2, 3]

        # rotation (quaternion)
        q = tf_transformations.quaternion_from_matrix(mat)
        obj.transform.rotation.x = q[0]
        obj.transform.rotation.y = q[1]
        obj.transform.rotation.z = q[2]
        obj.transform.rotation.w = q[3]

        return obj
    
    def node_callback(self):
        try:
            # Step 1: Get the current transform of the robot w.r.t. world
            tf_robot_to_world = self.tfBuffer.lookup_transform('world', 'base_link_gt', rclpy.time.Time())

            # Step 2: Convert the robot's transform to a 4x4 numpy array
            robot_to_world_matrix = self.tf_to_se3(tf_robot_to_world.transform)

            # Step 3: Compute the current transform of the left camera w.r.t. world
            tf_left_cam_matrix = np.dot(robot_to_world_matrix, self.left_cam_offset_to_matrix())

            # Step 4: Compute the current transform of the right camera w.r.t the left camera
            tf_right_cam_matrix = np.dot(tf_left_cam_matrix, self.right_cam_offset_to_matrix())

            # Step 5: Broadcast the computed transforms for the cameras to the TF tree
            tf_left_cam = self.se3_to_tf(tf_left_cam_matrix, tf_robot_to_world.header.stamp, 'world', 'left_cam')
            self.br.sendTransform(tf_left_cam)

            tf_right_cam = self.se3_to_tf(tf_right_cam_matrix, tf_robot_to_world.header.stamp, 'left_cam', 'right_cam')
            self.br.sendTransform(tf_right_cam)

            self.get_logger().info('Published')
        except tf2_ros.LookupException:
            self.get_logger().info('Failed to get transform')


    def left_cam_offset_to_matrix(self):
        return np.array([[1, 0, 0, self.left_cam_offset[0]],
                         [0, 1, 0, self.left_cam_offset[1]],
                         [0, 0, 1, self.left_cam_offset[2]],
                         [0, 0, 0, 1]])


    def right_cam_offset_to_matrix(self):
        return np.array([[1, 0, 0, self.right_cam_offset[0]],
                         [0, 1, 0, self.right_cam_offset[1]],
                         [0, 0, 1, self.right_cam_offset[2]],
                         [0, 0, 0, 1]])
    
def main(args=None):
    rclpy.init(args=args)
    dynamic_tf_cam_publisher = DynamicTFCamPublisher()
    rclpy.spin(dynamic_tf_cam_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

