#!/usr/bin/env python3

import os
import re

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class Path_Publisher(Node):

    def __init__(self):
        super().__init__("path_publisher")
        self.init_communications()
        self.setup_parameters()
        self.timer = self.create_timer(5.0, self.start_send)  # espera 5 segundos para que inicien los nodos

    def setup_parameters(self):
        self.declare_parameter("file", "path_line.txt") # literalmente el path del archivo que queramos que siga

    def init_communications(self):
        self.publisher = self.create_publisher(Path, "nav_plan", 10)

    def start_send(self):
        self.timer.cancel()
        nombre = self.get_parameter("file").value
        carpeta = get_package_share_directory("lab2")
        self.send_data(carpeta + "/text_files/" + nombre)

    def send_data(self, path):
        msg = Path()
        msg.header.frame_id = "odom"
        with open(path, "r") as file:
            for linea in file:
                datos = linea.strip().split(",")
                pose = PoseStamped()
                pose.pose.position.x = float(datos[0])
                pose.pose.position.y = float(datos[1])
                msg.poses.append(pose)
        self.publisher.publish(msg)
        self.get_logger().info("Trayectoria publicada: " + str(len(msg.poses)) + " puntos")


def main(args=None):
    rclpy.init(args=args)
    node = Path_Publisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()