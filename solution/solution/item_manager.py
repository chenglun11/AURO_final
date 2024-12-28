import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from assessment_interfaces.msg import Item,ItemList
from solution_interfaces.msg import PushItem,PushItemList
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class item_manager(Node):

    def __init__(self):
        super().__init__('item_manager')
        
        self.item_upstream_subscription = self.create_subscription(ItemList,'/robot1/items',self.item_callback, QoSProfile(depth = 10))
        self.item_publisher = self.create_publisher(PushItemList, '/fixed_items', QoSProfile(depth = 10))
        self.item_activation_subscription = self.create_subscription(PushItem,'/item_activate',self.item_activation_callback,QoSProfile(depth = 10))

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.item_publish)

        self.convert_item = []

    def item_callback(self,msg):
        for upstream_item in msg.data:
            item = PushItem()
            item.x = upstream_item.x
            item.y = upstream_item.y
            item.diameter = upstream_item.diameter    
            item.colour = upstream_item.colour
            item.value = upstream_item.value
            item.status = 'free'

            self.convert_item.append(item)

    def item_activation_callback(self,msg):
        

    def item_publish(self):
        item_list_msg = PushItemList()
        item_list_msg.data = self.convert_item
        self.item_publisher.publish(item_list_msg)
        self.get_logger().info(f"Converted and published {len(self.convert_item)} items.")
        self.convert_item = []

def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = item_manager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()