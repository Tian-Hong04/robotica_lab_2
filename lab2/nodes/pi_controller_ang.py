#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

def normalizar(angulo):
    # basicamente escoge el angulo mas corto en el que hay que rotar
    return np.arctan2(np.sin(angulo), np.cos(angulo))

class PIControllerAng(Node):

    def __init__(self, kp, ki = 0):
        super().__init__("pi_controller_ang")
        self.kp = kp
        self.ki = ki

        self.setpoint = None
        self.state = None
        self.integral_error = 0.0
        self.previous_time = None

        self.max_vel = 1.0   # Velocidad angular máxima para función de saturación

        self.actuation_pub = self.create_publisher(Float64, "ang/control_effort", 1)
        self.dist_set_point_sub = self.create_subscription(Float64, "ang/setpoint", self.setpoint_cb, 1)
        self.dist_state_sub = self.create_subscription(Float64, "ang/state", self.state_cb, 1)

        self.get_logger().info(f"Inicio controlador angular con Kp={self.kp}, ki={self.ki}")

    def setpoint_cb(self, msg):
        self.get_logger().info( '[PICTRL] new angular setpoint received: %.2f' % (msg.data) )
        self.reset()
        self.setpoint = normalizar(msg.data)

    def state_cb(self, msg):
        if self.setpoint == None:
            return
        self.state = msg.data
        error = normalizar(self.setpoint - self.state)

        now = self.get_clock().now().nanoseconds / 1e9

        # Calculo del tiempo transcurriddo (dt) dessde el último mensaje
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = now - self.previous_time
        self.previous_time = now

        # Proportional
        p_actuation = self.kp*error

        # Integrative
        if dt > 0.0:
            self.integral_error += error * dt
        i_actuation = self.ki * self.integral_error

        # Actuation
        actuation = p_actuation + i_actuation

        # Función de saturación
        actuation = max(-self.max_vel, min(self.max_vel, actuation))

        # Message sending
        msg = Float64()
        msg.data = actuation
        self.actuation_pub.publish(msg)

    def reset(self):
        self.setpoint = None
        self.state = None
        self.integral_error = 0.0
        self.previous_time = None

def main():
    rclpy.init()
    # Los valores que mejores resultados me dieron, kp = 17.0, ki = 0.01,
    # pueden seguir haciendo prueba y error para mejorarlos
    pi_ctrl_ang = PIControllerAng(17.0, ki=0.01)# Para probar solo controlador p, borren el ki
    rclpy.spin(pi_ctrl_ang)

if __name__ == '__main__':
    main()