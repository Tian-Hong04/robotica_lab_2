#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path, Odometry
from tf_transformations import euler_from_quaternion

class Follow_The_Carrot(Node):

    def __init__(self):
        super().__init__("follow_the_carrot")
        self.init_communications()
        self.setup_parameters()
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 veces por segundo

    def setup_parameters(self):
        self.lookahead = 0.4        # distancia de la zanahoria
        self.linear_speed = 0.1     # velocidad lineal fija
        self.goal_tolerance = 0.1   # distancia al final a la que nos detenemos

        self.path = []              # puntos x y de la trayectoria
        self.idx = 0                # indice del punto de la ruta mas cercano al robot
        self.done = False
        self.odo_x = 0.0
        self.odo_y = 0.0
        self.odo_yaw = 0.0
        self.odo_active = False
        self.real_x = 0.0
        self.real_y = 0.0
        self.real_yaw = 0.0
        self.real_active = False

    def init_communications(self):
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.path_sub = self.create_subscription(Path, "nav_plan", self.path_cb, 1)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.read_odometry, 1)
        self.real_sub = self.create_subscription(Pose, "/real_pose", self.read_real, 1)

        # Comunicacion con el controlador PID
        self.setpoint_pub = self.create_publisher(Float64, "heading/setpoint", 10)
        self.state_pub = self.create_publisher(Float64, "heading/state", 10)
        self.control_sub = self.create_subscription(Float64, "heading/control_effort", self.effort_cb, 10)

    def path_cb(self, msg: Path):
        # agregamos la parte real para no partir en 0 0
        self.path = []
        for pose in msg.poses:
            x = pose.pose.position.x - self.real_x + self.odo_x
            y = pose.pose.position.y - self.real_y + self.odo_y
            self.path.append((x, y))
        self.idx = 0
        self.done = False
        self.get_logger().info(f"camino a tomar en {len(self.path)} puntos")


    def read_odometry(self, data: Odometry):
        self.odo_x = data.pose.pose.position.x
        self.odo_y = data.pose.pose.position.y
        self.odo_yaw = euler_from_quaternion(
            (data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w))[2]
        self.odo_active = True

    def read_real(self, data: Pose):
        self.real_x = data.position.x
        self.real_y = data.position.y
        self.real_yaw = euler_from_quaternion(
            (data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w))[2]
        self.real_active = True





    def control_loop(self):
        if len(self.path) == 0 or not self.odo_active or self.done:
            return
        # 1. Punto de la ruta mas cercano al robot hacia adelante
        menor_dist = float("inf")
        for i in range(self.idx, len(self.path)):
            dist = math.hypot(self.path[i][0] - self.odo_x, self.path[i][1] - self.odo_y)
            if dist < menor_dist:
                menor_dist = dist
                self.idx = i
        final_x, final_y = self.path[-1] # detener el robot cuando llegamos al final
        if math.hypot(final_x - self.odo_x, final_y - self.odo_y) < self.goal_tolerance:
            self.done = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Trayectoria terminada")
            return
        x0, y0 = self.path[self.idx] # fijamos el punto que va a seguir
        j = self.idx
        while j < len(self.path) - 1 and math.hypot(self.path[j][0] - x0, self.path[j][1] - y0) < self.lookahead:
            j += 1
        carrot_x, carrot_y = self.path[j]
        angulo_zanahoria = math.atan2(carrot_y - self.odo_y, carrot_x - self.odo_x) # angulo entre la "zanahoria" y el angulo del robot
        msg = Float64()
        msg.data = angulo_zanahoria
        self.setpoint_pub.publish(msg)
        msg = Float64()
        msg.data = self.odo_yaw
        self.state_pub.publish(msg)

    def effort_cb(self, msg: Float64): # por ultimo enviamos las velocidades al robot
        if self.done:
            return
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = msg.data
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Follow_The_Carrot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()