#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rtabmap_msgs.msg import MapData

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_publishers = {}

    def handle_client_connection(self, client_socket, address):
        client_id = f"client_{address[0].replace('.', '_')}_{address[1]}"
        self.get_logger().info(f"Connected to client {client_id} at {address}")
        client_socket.settimeout(1800)

        try:
            while rclpy.ok():
                data = client_socket.recv(4096)
                if not data:
                    self.get_logger().info("No data received. Client may have closed the connection.")
                    break
                self.get_logger().info(f"Received data from client {client_id} at {address}")
                if data.startswith(b"START_STREAM") or data.startswith(b"STOP_STREAM"):
                    self.process_control_command(client_id, data.decode())
                else:
                    self.process_received_data(client_id, data)
        except socket.error as e:
            self.get_logger().error(f"Socket error with {client_id} at {address}: {e}")
        finally:
            client_socket.close()
            self.get_logger().info(f"Connection with {client_id} at {address} closed")
            self.cleanup_publishers(client_id)

    def process_control_command(self, client_id, data):
        try:
            parts = data.split(" ", 2)
            if len(parts) < 2:
                self.get_logger().error(f"Invalid control command format received: {data}")
                return

            command = parts[0]
            data_type = parts[1]

            self.get_logger().info(f"Processing control command: {command}, data_type: {data_type}")

            if command == "START_STREAM":
                self.start_stream(client_id, data_type)
            elif command == "STOP_STREAM":
                self.stop_stream(client_id, data_type)
            else:
                self.get_logger().warning(f"Unknown command: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to process control command: {e}")

    def process_received_data(self, client_id, data):
        try:
            data_type, serialized_data = data.split(b'|', 1)

            msg_class = self.get_message_class(data_type.decode())
            if not msg_class:
                self.get_logger().error(f"Unsupported data type: {data_type.decode()}")
                return

            msg = msg_class()
            msg.deserialize(serialized_data)

            topic_name = f"/received/{data_type.decode().lower()}/{client_id}"
            if topic_name not in self.tcp_publishers:
                self.tcp_publishers[topic_name] = self.create_publisher_for_type(data_type.decode(), topic_name)

            self.tcp_publishers[topic_name].publish(msg)
            self.get_logger().info(f"Published {data_type.decode()} to topic {topic_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to process received data: {e}")

    def start_stream(self, client_id, data_type):
        topic_name = f"/received/{data_type.lower()}/{client_id}"
        if topic_name not in self.tcp_publishers:
            self.tcp_publishers[topic_name] = self.create_publisher_for_type(data_type, topic_name)
        self.get_logger().info(f"Started streaming {data_type} to topic {topic_name}")

    def stop_stream(self, client_id, data_type):
        topic_name = f"/received/{data_type.lower()}/{client_id}"
        if topic_name in self.tcp_publishers:
            self.tcp_publishers.pop(topic_name)
            self.get_logger().info(f"Stopped streaming {data_type} from topic {topic_name}")
        else:
            self.get_logger().warning(f"Attempted to stop non-existent stream for {data_type} on topic {topic_name}")

    def get_message_class(self, data_type):
        if data_type == "RGB_IMAGE":
            return Image
        elif data_type == "DEPTH_IMAGE":
            return Image
        elif data_type == "POINT_CLOUD":
            return PointCloud2
        elif data_type == "LIDAR":
            return LaserScan
        elif data_type == "POSE":
            return PoseStamped
        elif data_type == "ODOMETRY":
            return Odometry
        elif data_type == "MAP_DATA":
            return MapData
        else:
            return None

    def create_publisher_for_type(self, data_type, topic_name):
        msg_class = self.get_message_class(data_type)
        if msg_class:
            return self.create_publisher(msg_class, topic_name, 10)
        else:
            self.get_logger().error(f"Unsupported data type: {data_type}")
            return None

    def cleanup_publishers(self, client_id):
        topics_to_remove = [topic for topic in self.tcp_publishers if client_id in topic]
        for topic in topics_to_remove:
            self.tcp_publishers.pop(topic, None)
            self.get_logger().info(f"Cleaned up publisher for topic: {topic}")

    def start_server(self):
        server_ip = '192.168.8.222'
        server_port = 11312
        self.server_socket.bind((server_ip, server_port))
        self.server_socket.listen(5)
        self.get_logger().info(f"Server is listening on port {server_port}")

        try:
            while rclpy.ok():
                self.get_logger().info("Waiting for a connection...")
                client_socket, addr = self.server_socket.accept()
                thread = threading.Thread(target=self.handle_client_connection, args=(client_socket, addr))
                thread.start()
        except KeyboardInterrupt:
            self.get_logger().info("Server is shutting down...")
        finally:
            self.server_socket.close()
            self.get_logger().info("Server socket closed.")

def main(args=None):
    rclpy.init(args=args)
    server_node = ServerNode()
    server_node.start_server()
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        server_node.get_logger().info("ROS node terminated")
    finally:
        server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

