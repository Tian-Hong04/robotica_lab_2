#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def movimiento_corto(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo)) # deja la direccion mas corta de rotacion

class PIDController(Node):
    def __init__(self, kp, ki, kd):
        super().__init__("pid_controller")
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = None
        self.integral_error = 0.0
        self.previous_error = None
        self.previous_time = None

        self.max_vel = 1.0 # velocidad angular maxima para que no se vuelva loco en caso de 

        self.actuation_pub = self.create_publisher(Float64, "heading/control_effort", 10)
        self.setpoint_sub = self.create_subscription(Float64, "heading/setpoint", self.setpoint_cb, 10)
        self.state_sub = self.create_subscription(Float64, "heading/state", self.state_cb, 10)
        self.get_logger().info(f"Inicio controlador PID con Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

    def setpoint_cb(self, msg):
        self.setpoint = msg.data

    def state_cb(self, msg):
        if self.setpoint == None:
            return
        error = movimiento_corto(self.setpoint - msg.data)

        # tiempo que paso desde el ultimo mensaje (dt)
        now = self.get_clock().now().nanoseconds / 1e9
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = now - self.previous_time
        self.previous_time = now

        # parte integral y derivativa
        derivada = 0.0
        if dt > 0.0:
            self.integral_error += error * dt
            derivada = movimiento_corto(error - self.previous_error) / dt
        self.previous_error = error

        # movimiento final
        final = self.kp * error + self.ki * self.integral_error + self.kd * derivada

        # limitador maximo
        actuation = max(-self.max_vel, min(self.max_vel, final))
        msg = Float64()
        msg.data = actuation
        self.actuation_pub.publish(msg)


def main():
    rclpy.init()
    pid_ctrl = PIDController(2.0, 0.0, 0.1)
    rclpy.spin(pid_ctrl)


if __name__ == '__main__':
    main()