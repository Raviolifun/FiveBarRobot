import time
import math
import numpy
import odrive
from odrive.enums import *
import sys

# setting path
sys.path.append("../parentdirectory")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, PointStamped
from Simulator.FiveBarDynamics import Dynamics


class OdriveDriver(Node):
    """Node Description"""

    def __init__(self):
        super().__init__("odrive_driver")

        self.subscription = self.create_subscription(
            msg_type=Joy, topic="joy", callback=self.joy_control, qos_profile=1
        )
        self.subscription2 = self.create_subscription(
            msg_type=PointStamped, topic="robot_pose", callback=self.robot_pose, qos_profile=1
        )
        self.subscription3 = self.create_subscription(
            msg_type=PointStamped, topic="puck_point", callback=self.puck_strike, qos_profile=1
        )
        self.joint_state_pub = self.create_publisher(msg_type=JointState, topic="joint_state", qos_profile=1)
        self.publisher = self.create_publisher(msg_type=Float64MultiArray, topic="encoder", qos_profile=1)
        self.robot_point = self.create_publisher(msg_type=PointStamped, topic="robot_point", qos_profile=1)
        # self.timer = self.create_timer(timer_period_sec = , callback = )

        self.get_logger().info("Finding all motors on drive")
        self.odrv0 = odrive.find_any()

        self.odrv0.axis0.motor.config.current_lim = 15
        self.odrv0.axis1.motor.config.current_lim = 15
        # self.odrv0.axis0.controller.config.vel_gain = 0.16
        # self.odrv0.axis0.controller.config.vel_integrator_gain = 0.32
        # self.odrv0.axis1.controller.config.vel_gain = 0.16
        # self.odrv0.axis1.controller.config.vel_integrator_gain = 0.32
        self.get_logger().warn("Completing startup calibration")
        # Commenting Out due to bugs
        # self.odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        # self.odrv0.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

        # while self.odrv0.axis0.current_state != AXIS_STATE_IDLE or self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
        #     time.sleep(0.1)
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE or self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        self.get_logger().info("Calibration complete")
        self.get_logger().warn("Setting Control as Closed Loop Control")

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.dt = 1 / 20.0
        self.x = 0.25
        self.y = 0.6

        self.flag = False
        self.flag2 = True
        self.joy_flag = False

        self.print_log = False
        self.enc_log = False
        self.theta_log = False

        mass_matrix = [0.048, 0.048, 0.074 + 0.050, 0.074 + 0.050]
        length_matrix = [0.163, 0.163, 0.450, 0.450]
        a_length_matrix = [0.335, 0.335, 0.6025, 0.6025]
        bottom_length = 0.4925

        self.dynamics = Dynamics(bottom_length, mass_matrix, length_matrix, a_length_matrix, 0.35, 0.38, 0)

        self.declare_parameter("theta_offset1", 0.0)
        self.declare_parameter("theta_offset2", 0.0)

    def joy_control(self, msg=Joy):
        """Simple Joy Mapper from x-y translation to joy axes"""
        # Callback Params
        self.joy_flag = True if msg.buttons[7] == 1 else False

        if self.joy_flag is False:
            return

        self.offset1 = self.get_parameter("theta_offset1").get_parameter_value().double_value
        self.offset2 = self.get_parameter("theta_offset2").get_parameter_value().double_value

        # Flags for soft E-Stops and Neutral
        self.flag = True if msg.buttons[8] == 1 else False
        self.flag2 = True if msg.buttons[4] == 0 else False

        self.print_log = True if msg.buttons[0] == 1 else False
        self.enc_log = True if msg.buttons[1] == 1 else False
        self.theta_log = True if msg.buttons[3] == 1 else False

        if self.flag is True:
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
            self.get_logger().warn("ODrive in Idle State")
            return
        else:
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        y_vel = round((msg.axes[1] / 8), 3)
        x_vel = round(((msg.axes[0] / 8) * -1.0), 2)

        self.x += round(x_vel * self.dt, 3)
        self.y += round(y_vel * self.dt, 3)

        self.x = min(self.x, 0.7)
        self.x = max(self.x, -0.2)
        self.y = min(self.y, 0.85)
        self.y = max(self.y, 0.25)

        self.x, self.y = self.dynamics.get_closest_solution(self.x, self.y, 0.01)
        theta1, theta2, theta3, theta4 = self.dynamics.inverse_kinematics(self.x, self.y)[0]

        if self.print_log is True:
            self.get_logger().info(f"x = {self.x}, y = {self.y}")

        if self.flag2 is True:
            theta1 = math.pi / 2
            theta2 = math.pi / 2
            self.get_logger().info("Setting To 90 Degrees")

        theta1 = -(theta1 + math.radians(self.offset1)) / (
            2 * math.pi
        )  # converting to rotations where 2pi is one rotation.
        theta2 = -(theta2 + math.radians(self.offset2)) / (2 * math.pi)

        if self.theta_log is True:
            self.get_logger().info(f"theta1 = {math.degrees(-theta1)}, theta2 = {math.degrees(-theta2)}")

        self.odrv0.axis0.controller.input_pos = theta1
        self.odrv0.axis1.controller.input_pos = theta2

        encoder1 = self.odrv0.axis0.encoder.pos_estimate
        encoder2 = self.odrv0.axis1.encoder.pos_estimate

        if self.enc_log is True:
            self.get_logger().info(f"Encoder1 = {encoder1}, Encoder2 = {encoder2}")

        msg_encoder = Float64MultiArray()
        msg_encoder.data = [encoder1, encoder2]
        self.publisher.publish(msg_encoder)
        msg_joint_state = JointState()
        msg_joint_state.position = [theta1, theta2, theta3, theta4]
        msg_joint_state.position = [self.x, self.y, theta1, theta2, theta3, theta4]
        msg_joint_state.name = ["EndPointPoseX", "EndPointPoseY", "Joint1", "Joint2", "Joint3", "Joint4"]
        self.joint_state_pub.publish(msg_joint_state)

    def robot_pose(self, msg=PointStamped):

        if self.joy_flag is True:
            return

        self.offset1 = self.get_parameter("theta_offset1").get_parameter_value().double_value
        self.offset2 = self.get_parameter("theta_offset2").get_parameter_value().double_value

        self.x = msg.point.x
        self.y = msg.point.y
        self.x = min(self.x, 0.7)
        self.x = max(self.x, -0.2)
        self.y = min(self.y, 0.85)
        self.y = max(self.y, 0.1)

        self.x, self.y = self.dynamics.get_closest_solution(self.x, self.y, 0.01)
        theta1, theta2, theta3, theta4 = self.dynamics.inverse_kinematics(self.x, self.y)[2]

        theta1 = -(theta1 + math.radians(self.offset1)) / (
            2 * math.pi
        )  # converting to rotations where 2pi is one rotation.
        theta2 = -(theta2 + math.radians(self.offset2)) / (2 * math.pi)

        self.odrv0.axis0.controller.input_pos = theta1
        self.odrv0.axis1.controller.input_pos = theta2

        encoder1 = self.odrv0.axis1.encoder.pos_estimate
        encoder2 = self.odrv0.axis1.encoder.pos_estimate

        msg_encoder = Float64MultiArray()
        msg_encoder.data = [encoder1, encoder2]
        self.publisher.publish(msg_encoder)
        msg_joint_state = JointState()
        msg_joint_state.position = [self.x, self.y, theta1, theta2, theta3, theta4]
        msg_joint_state.name = ["EndPointPoseX", "EndPointPoseY", "Joint1", "Joint2", "Joint3", "Joint4"]
        self.joint_state_pub.publish(msg_joint_state)

    def puck_strike(self, msg=PointStamped):
        if self.joy_flag is True:
            return

        self.offset1 = self.get_parameter("theta_offset1").get_parameter_value().double_value
        self.offset2 = self.get_parameter("theta_offset2").get_parameter_value().double_value

        self.x = msg.point.x
        self.y = msg.point.y
        self.x = min(self.x, 0.7)
        self.x = max(self.x, -0.2)
        self.y = min(self.y, 0.85)
        self.y = max(self.y, 0.15)

        # self.y = 0.5

        self.x, self.y = self.dynamics.get_closest_solution(self.x, self.y, 0.01)
        theta1, theta2, theta3, theta4 = self.dynamics.inverse_kinematics(self.x, self.y)[2]

        theta1 = -(theta1 + math.radians(self.offset1)) / (
            2 * math.pi
        )  # converting to rotations where 2pi is one rotation.
        theta2 = -(theta2 + math.radians(self.offset2)) / (2 * math.pi)

        self.odrv0.axis0.controller.input_pos = theta1
        self.odrv0.axis1.controller.input_pos = theta2

        encoder1 = self.odrv0.axis1.encoder.pos_estimate
        encoder2 = self.odrv0.axis1.encoder.pos_estimate

        msg_encoder = Float64MultiArray()
        msg_encoder.data = [encoder1, encoder2]
        self.publisher.publish(msg_encoder)
        msg_joint_state = JointState()
        msg_joint_state.position = [self.x, self.y, theta1, theta2, theta3, theta4]
        msg_joint_state.name = ["EndPointPoseX", "EndPointPoseY", "Joint1", "Joint2", "Joint3", "Joint4"]
        self.joint_state_pub.publish(msg_joint_state)
        msg_robot_point = PointStamped()
        msg_robot_point.point.x = self.x
        msg_robot_point.point.y = self.y
        msg_robot_point.header.frame_id = "robot"
        msg_robot_point.header.stamp = self.get_clock().now().to_msg()
        self.robot_point.publish(msg_robot_point)


def main(args=None):
    rclpy.init(args=args)

    odrive_driver = OdriveDriver()
    try:
        rclpy.spin(odrive_driver)
    except KeyboardInterrupt:
        odrive_driver.get_logger().warn("Setting Motor States to Idle, Shutting Down Node")
        odrive_driver.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        odrive_driver.odrv0.axis1.requested_state = AXIS_STATE_IDLE
    finally:
        odrive_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
