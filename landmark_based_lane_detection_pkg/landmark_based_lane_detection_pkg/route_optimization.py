# To find the shortest route between the pylones which still has 20cm to each pylon
# (assumption that the car is 30cm wide and 5cm to each side must be as margin)

import rclpy
from rclpy.node import Node

class route_optimization(Node):
    def __init__(self):
        super().__init__("route_optimization_node")
        self.get_logger().info("Route optimization node initialized.")


def main(args=None):
    """Entry point: initialize ROS2, create node, and spin."""
    rclpy.init(args=args)
    route_optimization_node = route_optimization()
    rclpy.spin(route_optimization_node)
    route_optimization_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
