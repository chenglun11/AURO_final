import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from solution_interfaces.msg import Zone,ZoneList
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class zone_manager(Node):

    def __init__(self):
        super().__init__('zone_manager')

        self.publisher = self.create_publisher(ZoneList, '/fixed_zones', QoSProfile(depth = 10))
        self.zone_activation_subscription = self.create_subscription(Zone,'/zone_activate',self.zone_activation_callback,QoSProfile(depth = 10))
        self.zones = {
            'TOP_RIGHT': {'c':'None','wx': 2.35, 'wy': -2.5,'ww':1.0,'status':'free'},      # 右上角
            'TOP_LEFT': {'c':'None','wx': 2.35, 'wy': 2.35,'ww':1.0,'status':'free'},     # 左上角
            'BUT_LEFT': {'c':'None','wx': -3.25, 'wy': 2.35,'ww':1.0,'status':'free'}, # 左下角
            'BUT_RIGHT': {'c':'None','wx': -3.25, 'wy': -2.35,'ww':1.0,'status':'free'}       # 左下角
        }

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_zones)

    def publish_zones(self):
        msg = ZoneList()
        msg.zones = []
        for name, data in self.zones.items():
            try:
                zone = Zone()
                zone.name = name
                zone.wx = float(data['wx'])
                zone.wy = float(data['wy'])
                zone.ww = float(data['ww'])
                zone.fixed_colour = data['c'] if data['c'] is not None else 'None'
                zone.status = data['status'] if data['status'] is not None else 'unknown'
                msg.zones.append(zone)
            except KeyError as e:
                self.get_logger().error(f"Zone data for {name} is missing key: {e}")
            except ValueError as e:
                self.get_logger().error(f"Zone data for {name} has invalid value: {e}")

        self.publisher.publish(msg)
        #self.get_logger().info(f"Published {msg}")

    def zone_activation_callback(self,msg):
        if msg.name in self.zones:
            if self.zones[msg.name]['c'] is None:
                self.zones[msg.name]['c'] = msg.fixed_colour
                self.get_logger().info(f"Zone {msg.name} activate colour {msg.fixed_colour}")
            else:
                self.get_logger().info(f"Zone {msg.name} is already activate colour {self.zones[msg.name]['c']}")
        else:
            self.get_logger().info(f"Zone {msg.name} is not exist")


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = zone_manager()

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