import rclpy
from rclpy.node import Node

class pylon_detection(Node):
    def __init__(self):
        super().__init__("pylon_detection_node")
        self.get_logger().info("Pylon detection node initialized.")


def main(args=None):
    """Entry point: initialize ROS2, create node, and spin."""
    rclpy.init(args=args)
    pylon_detection_node = pylon_detection()
    rclpy.spin(pylon_detection_node)
    pylon_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
