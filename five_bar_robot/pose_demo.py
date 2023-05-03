import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, String
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
import numpy as np


class PathDemos:
    "Class for different paths; Figure 8, other paths etc."

    def __init__(self, t_size):

        self.weights1 = [3, 3]
        self.weights2 = [3, 3]
        self.t_step = np.linspace(0, 1, t_size)

    def bezier(self, t_step, points_matrix, weights_vector):

        x = (
            points_matrix[0, 0] * (1 - t_step) ** 3
            + points_matrix[1, 0] * weights_vector[0] * (1 - t_step) ** 2 * t_step
            + points_matrix[2, 0] * weights_vector[1] * (1 - t_step) ** 1 * t_step**2
            + points_matrix[3, 0] * t_step**3
        ).reshape((np.shape(t_step)[0], 1))
        y = (
            points_matrix[0, 1] * (1 - t_step) ** 3
            + points_matrix[1, 1] * weights_vector[0] * (1 - t_step) ** 2 * t_step
            + points_matrix[2, 1] * weights_vector[1] * (1 - t_step) ** 1 * t_step**2
            + points_matrix[3, 1] * t_step**3
        ).reshape((np.shape(t_step)[0], 1))

        return np.hstack((x, y))

    def figure_8(self):

        figure8_pose_matrix = np.array([[0.25, 0.74], [-0.5, 0.5], [1.0, 0.3], [0.25, 0.2], [-0.5, 0.3], [1.0, 0.5]])

        points1_matrix = figure8_pose_matrix[0:4, :]

        points2_matrix = np.append(figure8_pose_matrix[3:, :], [figure8_pose_matrix[0, :]], axis=0)
        figure8_cont = np.append(
            self.bezier(t_step=self.t_step.T, points_matrix=points1_matrix, weights_vector=self.weights1),
            self.bezier(t_step=self.t_step.T, points_matrix=points2_matrix, weights_vector=self.weights2),
            axis=0,
        )
        return figure8_cont[:, 0], figure8_cont[:, 1]

    def inverted_w(self):
        w_pose = np.array(
            [
                [0.25, 0.9],
                [-0.25, 0.8],
                [-0.25, 0.7],
                [0.25, 0.6],
                [0.25, 0.6],
                [-0.25, 0.5],
                [-0.25, 0.4],
                [0.25, 0.3],
                [0.25, 0.3],
                [0.75, 0.4],
                [0.75, 0.5],
                [0.25, 0.6],
                [0.25, 0.6],
                [0.75, 0.7],
                [0.75, 0.8],
                [0.25, 0.9],
            ]
        )
        points_1 = w_pose[0:4, :]
        points_2 = w_pose[4:8, :]
        points_3 = w_pose[8:12, :]
        points_4 = w_pose[12:17, :]

        w_pose_cont = np.vstack(
            (
                self.bezier(t_step=self.t_step.T, points_matrix=points_1, weights_vector=self.weights1),
                self.bezier(t_step=self.t_step.T, points_matrix=points_2, weights_vector=self.weights2),
                self.bezier(t_step=self.t_step.T, points_matrix=points_3, weights_vector=self.weights1),
                self.bezier(t_step=self.t_step.T, points_matrix=points_4, weights_vector=self.weights2),
            )
        )
        return w_pose_cont[:, 0], w_pose_cont[:, 1]


class PoseDemo(Node):
    """Node Description"""

    def __init__(self):
        super().__init__("pose_demo")

        # self.subscription = self.create_subscription(msg_type = , topic = , callback = , qos_profile = 1)
        self.publisher = self.create_publisher(msg_type=PointStamped, topic="robot_pose", qos_profile=1)
        self.publisher2 = self.create_publisher(msg_type=Path, topic="robot_path", qos_profile=1)
        self.timer = self.create_timer(timer_period_sec=1 / 1000, callback=self.robot_pose_demo)
        self.declare_parameter("Pose", "figure8")
        self.pose_type_last = "invertedW"
        self.x = []
        self.y = []
        t_size = 10000
        self.counter = 0
        pose_demo = PathDemos(t_size=t_size)
        self.x8, self.y8 = pose_demo.figure_8()
        self.x_w, self.y_w = pose_demo.inverted_w()

        self.get_logger().warn("Building Paths for Rviz")

        self.msg_figure8 = Path()
        for i, _ in enumerate(self.x8):
            msg_list = PoseStamped()
            msg_list.pose.position.x = self.x8[i]
            msg_list.pose.position.y = self.y8[i]
            msg_list.pose.position.z = 0.0
            self.msg_figure8.poses.append(msg_list)
        self.msg_figure8.header.stamp = self.get_clock().now().to_msg()
        self.msg_figure8.header.frame_id = "robot"
        self.msg_invertedW = Path()

        for i, _ in enumerate(self.x_w):
            msg_list = PointStamped()
            msg_list.point.x = self.x_w[i]
            msg_list.point.y = self.y_w[i]
            msg_list.point.z = 0.0
            self.msg_invertedW.poses.append(msg_list)
        self.msg_invertedW.header.stamp = self.get_clock().now().to_msg()
        self.msg_invertedW.header.frame_id = "robot"
        self.get_logger().warn("Completed Paths for Rviz")

    def robot_pose_demo(self):
        """Description of Callback"""
        self.pose_type = self.get_parameter("Pose").get_parameter_value().string_value
        if self.pose_type == "figure8" and self.pose_type_last != self.pose_type:
            self.x = self.x8
            self.y = self.y8
            self.publisher2.publish(self.msg_figure8)
        elif self.pose_type == "invertedW" and self.pose_type_last != self.pose_type:
            self.x = self.x_w
            self.y = self.y_w
            self.publisher2.publish(self.msg_invertedW)

        msg_out = PoseStamped()
        msg_out.header.frame_id = "robot"
        msg_out.header.stamp = self.get_clock().now().to_msg()

        msg_out.pose.position.x = self.x[self.counter]
        msg_out.pose.position.y = self.y[self.counter]

        self.counter += 1

        self.counter = 0 if self.counter >= len(self.x) else self.counter

        self.publisher.publish(msg_out)
        self.pose_type_last = self.pose_type


def main(args=None):
    rclpy.init(args=args)
    pose_demo = PoseDemo()
    # executor = MultiThreadedExecutor()
    # executor.add_node(pose_demo)
    # executor.spin()

    rclpy.spin(pose_demo)
    pose_demo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
