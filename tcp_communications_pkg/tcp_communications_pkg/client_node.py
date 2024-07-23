#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rtabmap_ros.msg import MapData
import os

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.subscribers = {}

    def handle_server_communication(self):
        try:
            # Continuously receive data from the server
            while rclpy.ok():
                response = self.client_socket.recv(1024)
                if not response:
                    break  # If no data is received, exit the loop
                self.get_logger().info(f"Received from server: {response.decode()}")
                self.handle_server_command(response.decode())

        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")
        finally:
            self.client_socket.close()
            self.get_logger().info("Socket closed.")

    def handle_server_command(self, command):
        # Process commands to start/stop streaming or send one-time snippets
        if command.startswith("START_STREAM"):
            data_type = command.split()[1]
            self.start_stream(data_type)
        elif command.startswith("STOP_STREAM"):
            data_type = command.split()[1]
            self.stop_stream(data_type)
        elif command.startswith("SEND_SNIPPET"):
            data_type = command.split()[1]
            source = command.split()[2] if len(command.split()) > 2 else None
            self.send_snippet(data_type, source)

    def start_stream(self, data_type):
        if data_type == "RGB_IMAGE":
            self.subscribers["RGB_IMAGE"] = self.create_subscription(
                Image, "/camera/color/image_raw", self.send_rgb_image_stream, 5)
        elif data_type == "DEPTH_IMAGE":
            self.subscribers["DEPTH_IMAGE"] = self.create_subscription(
                Image, "/camera/depth/image_rect_raw", self.send_depth_image_stream, 5)
        elif data_type == "POINT_CLOUD":
            self.subscribers["POINT_CLOUD"] = self.create_subscription(
                PointCloud2, "/camera/depth/color/points", self.send_point_cloud_stream, 5)
        elif data_type == "LIDAR":
            self.subscribers["LIDAR"] = self.create_subscription(
                LaserScan, "/scan", self.send_lidar_stream, 10)
        elif data_type == "POSE":
            self.subscribers["POSE"] = self.create_subscription(
                PoseStamped, "/pose", self.send_pose_stream, 10)
        elif data_type == "ODOMETRY":
            self.subscribers["ODOMETRY"] = self.create_subscription(
                Odometry, "/odom", self.send_odometry_stream, 10)
        elif data_type == "MAP_DATA":
            self.subscribers["MAP_DATA"] = self.create_subscription(
                MapData, "/mapData", self.send_map_data_stream, 10)
        self.get_logger().info(f"Started streaming {data_type}")

    def stop_stream(self, data_type):
        if data_type in self.subscribers:
            self.destroy_subscription(self.subscribers[data_type])
            del self.subscribers[data_type]
            self.get_logger().info(f"Stopped streaming {data_type}")

    def send_rgb_image_stream(self, data):
        # Send RGB image data to the server
        self.get_logger().info("Streaming RGB image")

    def send_depth_image_stream(self, data):
        # Send depth image data to the server
        self.get_logger().info("Streaming depth image")

    def send_point_cloud_stream(self, data):
        # Send point cloud data to the server
        self.get_logger().info("Streaming point cloud")

    def send_lidar_stream(self, data):
        # Send Lidar data to the server
        self.get_logger().info("Streaming Lidar scan")

    def send_pose_stream(self, data):
        # Send pose data to the server
        self.get_logger().info("Streaming pose data")

    def send_odometry_stream(self, data):
        # Send odometry data to the server
        self.get_logger().info("Streaming odometry data")

    def send_map_data_stream(self, data):
        # Send map data to the server
        self.get_logger().info("Streaming map data")

    def send_snippet(self, data_type, source):
        if source is None:
            # Send data from the current ROS topic
            if data_type == "RGB_IMAGE":
                self.subscribers["RGB_IMAGE_SNIPPET"] = self.create_subscription(
                    Image, "/camera/rgb/image_raw", self.send_rgb_image_snippet, 10)
            elif data_type == "DEPTH_IMAGE":
                self.subscribers["DEPTH_IMAGE_SNIPPET"] = self.create_subscription(
                    Image, "/camera/depth/image_raw", self.send_depth_image_snippet, 10)
            elif data_type == "POINT_CLOUD":
                self.subscribers["POINT_CLOUD_SNIPPET"] = self.create_subscription(
                    PointCloud2, "/camera/depth/points", self.send_point_cloud_snippet, 10)
            elif data_type == "LIDAR":
                self.subscribers["LIDAR_SNIPPET"] = self.create_subscription(
                    LaserScan, "/scan", self.send_lidar_snippet, 10)
            elif data_type == "POSE":
                self.subscribers["POSE_SNIPPET"] = self.create_subscription(
                    PoseStamped, "/pose", self.send_pose_snippet, 10)
            elif data_type == "ODOMETRY":
                self.subscribers["ODOMETRY_SNIPPET"] = self.create_subscription(
                    Odometry, "/odom", self.send_odometry_snippet, 10)
            elif data_type == "MAP_DATA":
                self.subscribers["MAP_DATA_SNIPPET"] = self.create_subscription(
                    MapData, "/mapData", self.send_map_data_snippet, 10)
            self.get_logger().info(f"Sending one-time snippet of {data_type} from topic")
        else:
            # Send data from the specified file
            self.get_logger().info(f"Sending one-time snippet of {data_type} from file: {source}")
            self.send_file_snippet(data_type, source)

    def send_rgb_image_snippet(self, data):
        # Send RGB image data to the server
        self.get_logger().info("Sending RGB image snippet")
        # Unsubscribe after sending one snippet
        self.destroy_subscription(self.subscribers["RGB_IMAGE_SNIPPET"])

    def send_depth_image_snippet(self, data):
        # Send depth image data to the server
        self.get_logger().info("Sending depth image snippet")
        # Unsubscribe after sending one snippet
        self.destroy_subscription(self.subscribers["DEPTH_IMAGE_SNIPPET"])

    def send_point_cloud_snippet(self, data):
        # Send point cloud data to the server
        self.get_logger().info("Sending point cloud snippet")
        # Unsubscribe after sending one snippet
        self.destroy_subscription(self.subscribers["POINT_CLOUD_SNIPPET"])

    def send_lidar_snippet(self, data):
        # Send Lidar data to the server
        self.get_logger().info("Sending Lidar snippet")
        # Unsubscribe after sending one snippet
        self.destroy_subscription(self.subscribers["LIDAR_SNIPPET"])

    def send_pose_snippet(self, data):
        # Send pose data to the server
        self.get_logger().info("Sending pose snippet")
        # Unsubscribe after sending one snippet
        self.destroy_subscription(self.subscribers["POSE_SNIPPET"])

    def send_odometry_snippet(self, data):
        # Send odometry data to the server
        self.get_logger().info("Sending odometry snippet")
        # Unsubscribe after sending one snippet
        self.destroy_subscription(self.subscribers["ODOMETRY_SNIPPET"])

    def send_map_data_snippet(self, data):
        # Send map data to the server
        self.get_logger().info("Sending map data snippet")
        # Unsubscribe after sending one snippet
        self.destroy_subscription(self.subscribers["MAP_DATA_SNIPPET"])

    def send_file_snippet(self, data_type, source):
        # Implement logic to read file and send its content to the server
        if data_type == "2D_LIDAR_MAP":
            pgm_file = f"{source}.pgm"
            yaml_file = f"{source}.yaml"
            if os.path.exists(pgm_file) and os.path.exists(yaml_file):
                with open(pgm_file, 'rb') as pgm, open(yaml_file, 'r') as yaml:
                    pgm_data = pgm.read()
                    yaml_data = yaml.read()
                    self.get_logger().info(f"Sending {pgm_file} and {yaml_file}")
                    self.client_socket.sendall(pgm_data)
                    self.client_socket.sendall(yaml_data.encode())
            else:
                self.get_logger().error(f"Files {pgm_file} or {yaml_file} do not exist")

    def connect_to_server(self):
        server_ip = '192.168.1.11'  # Change to your server's IP
        server_port = 11312         # Change to your server's listening port

        try:
            self.client_socket.connect((server_ip, server_port))
            self.get_logger().info(f"Connected to server at {server_ip}:{server_port}")

            # Start a new thread for handling server communication
            thread = threading.Thread(target=self.handle_server_communication)
            thread.start()

        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")
            self.client_socket.close()

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    client_node.connect_to_server()
    try:
        rclpy.spin(client_node)
    except KeyboardInterrupt:
        client_node.get_logger().info("Node terminated")
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
