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
from nav2_simple_commander.robot_navigator import TaskResult

from solution_interfaces.msg import Zone, ZoneList, PushItem, PushItemList
from rclpy.duration import Duration

from tf_transformations import euler_from_quaternion

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.odom_subscriber = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.item_subscriber = self.create_subscription(PushItemList, '/fixed_items', self.item_callback, 10)
        self.zone_subscriber = self.create_subscription(ZoneList, '/fixed_zones', self.zone_callback, 10)
        self.zone_activation_publisher = self.create_publisher(Zone, '/zone_activate', 10)
        self.item_activation_publisher = self.create_publisher(PushItem,'/item_activate',10)

        self.current_items = []
        self.current_zones = {}

        self.declare_parameter('x', -3.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('robot_id', 'robot1')

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.robot_id = self.get_parameter('robot_id').value

        self.pos_x = self.initial_x
        self.pos_y = self.initial_y
        self.yaw = self.initial_yaw
        self.item_held = None
        self.loop = 1

        self.get_logger().info(f"\tRobot Controller Initialized for {self.robot_id}\t")

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        self.pos_x = msg.pose.pose.position.x + self.initial_x
        self.pos_y = msg.pose.pose.position.y + self.initial_y
        self.yaw = math.degrees(yaw) % 360

    def item_callback(self, msg):
        self.current_items = msg.data

    def zone_callback(self, msg):
        self.current_zones = {
            zone.name: {
                'wx': zone.wx,
                'wy': zone.wy,
                'ww': zone.ww,
                'colour': zone.fixed_colour,
                'status': zone.status
            } for zone in msg.zones
        }
        self.get_logger().info(f"Updated Zones: {self.current_zones}")

    def find_closest_item(self):
        if not self.current_items:
            self.get_logger().info("No items available.")
            return None

        closest_item = None
        min_distance = float('inf')

        for item in self.current_items:
            if item.status != 'free':
                continue

            distance = math.sqrt((item.x - self.pos_x)**2 + (item.y - self.pos_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_item = item

        if closest_item:
            self.get_logger().info(f"Closest item: {closest_item.id} at distance {min_distance:.2f}")
        else:
            self.get_logger().info("No free items found.")

        return closest_item

    def activate_item(self):
        closest_item = self.find_closest_item()

        if not closest_item:
            return

        activation_msg = PushItem(
            id=closest_item.id,
            x=closest_item.x,
            y=closest_item.y,
            diameter=closest_item.diameter,
            colour=closest_item.colour,
            value=closest_item.value,
            status='assigned'
        )

        self.item_activation_publisher.publish(activation_msg)
        self.get_logger().info(f"Activated item {closest_item.id}")
        closest_item.status = 'assigned'
        time.sleep(10)

    def activate_zone(self, zone_name, colour):
        zone_msg = Zone(
            name=zone_name,
            fixed_colour=colour,
            status='assigned'
        )

        self.zone_activation_publisher.publish(zone_msg)
        self.get_logger().info(f"Activated zone {zone_name} for colour {colour}")

    def check_zone_available(self, colour):
        for zone_name, zone_data in self.current_zones.items():
            if zone_data['colour'] == colour:
                self.get_logger().info(f"Zone {zone_name} already assigned to colour {colour}")
                return zone_name

        for zone_name, zone_data in self.current_zones.items():
            if zone_data['colour'] is None:
                self.activate_zone(zone_name, colour)
                return zone_name

        self.get_logger().error(f"No available zones for colour {colour}")
        return None

    def control_loop(self):
        self.activate_item()


    def destroy_node(self):
        self.get_logger().info("Robot Controller shutting down.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
