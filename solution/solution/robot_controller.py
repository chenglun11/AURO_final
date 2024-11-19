import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from auro_interfaces.srv import ItemRequest
from assessment_interfaces.msg import ItemList, ZoneList


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.pick_up_client = self.create_client(ItemRequest, '/pick_up_item')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')

        self.item_subscriber = self.create_subscription(ItemList, 'items', self.item_callback, 10)
        self.zone_subscriber = self.create_subscription(ZoneList, 'zone', self.zone_callback, 10)

        self.current_items = []
        self.current_zones = []

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def item_callback(self,msg):
        self.current_items = msg.data

    def zone_callbake(self,msg):
        self.current_zones = msg.data;

    def item_pickup(self):
        requests = ItemRequest.Request()
        robot_id = requests.robot_id
        

    def control_loop(self):

        self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")


    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()