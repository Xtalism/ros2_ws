import sys, rclpy, os

from PyQt5.QtCore import QThread

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

PROJECT_TOPIC = '/gui/cmd'

class RosNode(Node):
    def __init__(self, topic=PROJECT_TOPIC):
        super().__init__('gui_node')
        self.apublisher = self.create_publisher(String, topic, 10)

    def publish(self, key):
        msg = String()
        msg.data = key
        self.apublisher.publish(msg)
        self.get_logger().info(f'GUI PUB -> {msg.data}')

class RosThread(QThread):
    def __init__(self, topic=PROJECT_TOPIC):
        super().__init__()
        rclpy.init(args=None)
        self.node = RosNode(topic)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

    def run(self):
        self.executor.spin()

    def stop(self):
        self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.quit()
        self.wait()
