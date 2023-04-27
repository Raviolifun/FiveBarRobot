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
from Simulator.FiveBarDynamics import Dynamics


class OdriveDriver(Node):
    """Node Description"""

    def __init__(self):
        super().__init__("odrive_driver")

        self.subscription = self.create_subscription(msg_type=Joy, topic="joy", callback=self.joy_control, qos_profile=1)
        self.subscription2 = self.create_subscription(msg_type=JointState, topic="joint_state", callback=self.joint_state, qos_profile=1)
        self.publisher = self.create_publisher(msg_type=Float64MultiArray, topic="encoder", qos_profile=1)
        # self.timer = self.create_timer(timer_period_sec = , callback = )

        self.get_logger().info("Finding all motors on drive")
        self.odrv0 = odrive.find_any()
        self.get_logger().warn("Completing startup calibration")
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
        self.y = 0.5

        self.flag = False
        self.flag2 = True
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
            return
        else:
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        y_vel = round((msg.axes[1] / 8), 3)
        x_vel = round(((msg.axes[0] / 8) * -1.0), 2)

        self.x += round(x_vel * self.dt, 3)
        self.y += round(y_vel * self.dt, 3)

        self.x = min(self.x, 0.49)
        self.x = max(self.x, 0)
        self.y = min(self.y, 0.75)
        self.y = max(self.y, 0.35)

        self.x, self.y = self.dynamics.get_closest_solution(self.x, self.y, 0.01)
        theta1, theta2, _, _ = self.dynamics.inverse_kinematics(self.x, self.y)[0]

        if self.print_log is True:
            self.get_logger().info(f"x = {self.x}, y = {self.y}")

        if self.flag2 is True:
            theta1 = 0.0
            theta2 = 0.0
            return

        theta1 = theta1 / (2 * math.pi) + math.radians(self.offset1)  # converting to rotations where 2pi is one rotation.
        theta2 = (theta2 / (2 * math.pi)) + math.radians(self.offset2)

        if self.theta_log is True:
            self.get_logger().info(f"theta1 = {math.degrees(theta1)}, theta2 = {math.degrees(theta2)}")

        self.odrv0.axis0.controller.input_pos = theta1
        self.odrv0.axis1.controller.input_pos = theta2

        encoder1 = self.odrv0.axis1.encoder.pos_estimate
        encoder2 = self.odrv0.axis1.encoder.pos_estimate

        if self.enc_log is True:
            self.get_logger().info(f"Encoder1 = {encoder1}, Encoder2 = {encoder2}")

        msg_encoder = Float64MultiArray()
        msg_encoder.data = [encoder1, encoder2]
        self.publisher.publish(msg_encoder)

    def joint_state(self, msg=JointState):
        """Publishing position joint states of the 5-Bar Mechanism"""
        # You need replace this


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
