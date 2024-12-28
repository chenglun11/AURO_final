from enum import IntEnum
import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from nav2_msgs.action import Spin 

from nav2_simple_commander.robot_navigator import TaskResult

# from assessment_interfaces.msg import  ZoneList,ItemList
from solution_interfaces.msg import Zone, ZoneList, PushItemList
from rclpy.duration import Duration

from tf_transformations import euler_from_quaternion

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')


        self.item_subscriber = self.create_subscription(PushItemList, '/fixed_items', self.item_callback, 10)
        self.zone_subscriber = self.create_subscription(ZoneList, '/fixed_zones', self.zone_callback, 10)
        self.zone_activation_publisher = self.create_publisher(Zone, '/zone_activate', 10)
        #self.zone_activation_publisher = self.create_publisher(Zone,'/zone_activate',self.activate_zone,10)
        self.current_items = []
        self.current_zones = {}

        self.declare_parameter('x', -3.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('robot_id', 'robot1')

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y =  self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.robot_id = self.get_parameter('robot_id').value

        self.loop = 1
        self.item_held = None
        self.nav_to_ball_flag = False
        self.nav_to_zone_flag = False
        self.current_target = {}
        self.pos_x = self.initial_x
        self.pos_y = self.initial_y
        self.yaw =  self.initial_yaw
        self.get_logger().info(f"\t初始化完成\t")


        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def item_callback(self,msg):
        #self.get_logger().info(f"Received {len(msg.data)} items from ItemSensor.")
        self.current_items = msg.data # 存储物品列表

    def zone_callback(self,msg):
        # self.get_logger().info(f"Received {len(msg.data)} items from ZoneSensor.")
        #self.current_zones = msg.data
        self.current_zones = {zone.name:{'wx':zone.wx,'wy':zone.wy,'ww':zone.ww,'c':zone.fixed_colour,'status':zone.status}for zone in msg.zones}
        self.get_logger().info(f"Update zones{self.current_zones}")
        
    def find_ball_position(self, ball):
        current_target = {}
        estimated_distance = (69.0 * (float(ball.diameter) ** -0.89)) + 0.1 #aims further then nessessary
        ball_x_mult = (0.003 * estimated_distance) + 0.085 #since camera is mounted near front mult changes with distance
        angle_diff = -ball_x_mult * ball.x
        estimated_angle = self.yaw + angle_diff
            
        x = estimated_distance * math.cos(math.radians(estimated_angle)) #x is positive towards 0 degrees?!
        y = estimated_distance * -math.sin(math.radians(estimated_angle)) #y is positive towards 90 degrees?!

        current_target['colour'] = ball.colour
        current_target['wx'] = x + (self.pos_x)
        current_target['wy'] = y + self.pos_y
        current_target['ww'] = estimated_angle
        self.get_logger().info(f"Item.x{x:2f}, Item.y{y:2f},Item.w{estimated_angle:2f}")
        self.get_logger().info(f"currect_item:{current_target}")
        return current_target
        # 返回至enact

    def find_closest_item(self):
        if not self.current_items:
            self.get_logger().info("No items available to find the closest one.")
            return None

        min_distance = float('inf')
        closest_item = None

        for item in self.current_items:
            # 计算物品到机器人的距离
            distance = math.sqrt(item.x ** 2 + item.y ** 2)
            self.get_logger().info(
                f"Item - Colour: {item.colour}, Distance: {distance:.2f}"
            )
            # 更新最小距离和最近物品
            if distance < min_distance:
                min_distance = distance
                closest_item = item

        self.get_logger().info(
            f"Closest item: {closest_item}"
        )
        return closest_item


    def activate_zone(self,zone_name,colour):
        zone_msg = Zone()
        zone_msg.name = zone_name
        zone_msg.fixed_colour = colour
        zone_msg.status = 'assigned'
        self.zone_activation_publisher.publish(zone_msg)
        self.get_logger().info(f'Published: {zone_msg.name}, {zone_msg.fixed_colour}')

                 
    def check_zone_available(self, colour):
        """
        查找并分配目标颜色的区域。
        如果找到已有颜色的区域，直接返回。
        如果没有找到，则分配目标颜色到第一个未分配的区域。
        最终返回一个包含分配或找到区域的 `checked_zone`。
        """
        checked_zone = {}  # 用于记录被修改的区域
        #zone_names = random.sample(list(self.zones.keys()), len(self.currect_zones))  # 随机化顺序
        zone_names = random.sample(list(self.current_zones.keys()),len(self.current_zones))
        for zone_name in zone_names:  # 使用随机化后的区域顺序
            zone_data = self.zones[zone_name]
            
            # 优先查找相同颜色的区域
            if zone_data['c'] == colour:  # 如果找到已有颜色
                #
                self.get_logger().info(f"Found existing zone for colour '{colour}': {zone_name}")
                return checked_zone  # 立即返回

        # 如果没有找到相同颜色，再查找空闲区域
        for zone_name in zone_names:  # 再次遍历区域
            zone_data = self.zones[zone_name]
            if zone_data['c'] is None:  # 如果找到空闲区域
                self.activate_zone(zone_name,colour)
                checked_zone[zone_name] = zone_data
                self.get_logger().info(f"Assigned colour '{colour}' to zone '{zone_name}'")
                return checked_zone  # 返回分配的区域

        # 如果没有找到可用区域
        self.get_logger().error(f"No available zone for colour '{colour}'.")
        return checked_zone  # 返回空的 `checked_zone`

    def control_loop(self):
        #self.print_zone()
        #self.activate_zone('TOP_RIGHT','RED')
        #self.check_zone_available('GREEN')
        self.print_item()

    def print_item(self):
        for item in self.current_items:
            self.get_logger().info(f"Item received: {item}")

    def print_zone(self):
        for zone_name, zone_data in self.current_zones.items():
            self.get_logger().info(f"Zone: {zone_name}")
            for key, value in zone_data.items():
                self.get_logger().info(f"  {key}: {value}")
        self.get_logger().info("-" * 30)  # 分隔线
        
    def destroy_node(self):
        self.get_logger().info("Shutting down spin action server...")
    # 确保所有目标被正确取消
        if hasattr(self, '_action_server') and self._action_server:
            self._action_server.destroy()
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