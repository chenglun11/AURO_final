import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from threading import Lock
from assessment_interfaces.msg import ItemList
from solution_interfaces.msg import PushItem, PushItemList
from nav_msgs.msg import Odometry

class ItemManager(Node):
    def __init__(self):
        super().__init__('item_manager')
        self.robot_count = self.declare_parameter('robot_count', 2).get_parameter_value().integer_value

        self.items = []  # 当前物品列表
        self.robot_positions = {}  # 每个机器人的位置 {robot_id: (x, y)}
        self.lock = Lock()  # 确保线程安全
        self.robot_publishers = {}  # 每个机器人分配目标的发布器

        # 初始化每个机器人的订阅和发布
        for robot_id in range(1, self.robot_count + 1):
            self.setup_robot(robot_id)

        # 定时器用于检查并分配物品
        self.create_timer(1.0, self.assign_items_to_robots)

    def setup_robot(self, robot_id):
        """
        设置每个机器人的订阅和发布
        """
        robot_namespace = f'/robot{robot_id}'

        # 订阅物品列表
        self.create_subscription(
            ItemList,
            f'{robot_namespace}/items',
            lambda msg, rid=robot_id: self.item_callback(msg, rid),
            QoSProfile(depth=10)
        )

        # 订阅机器人位置
        self.create_subscription(
            Odometry,
            f'{robot_namespace}/odom',
            lambda msg, rid=robot_id: self.update_robot_position(msg, rid),
            QoSProfile(depth=10)
        )

        # 创建发布器
        publisher = self.create_publisher(
            PushItemList,
            f'{robot_namespace}/assign_items',
            QoSProfile(depth=10)
        )
        self.robot_publishers[robot_id] = publisher

    def update_robot_position(self, msg, robot_id):
        """
        更新机器人位置
        """
        with self.lock:
            position = msg.pose.pose.position
            self.robot_positions[robot_id] = (position.x, position.y)

    def item_callback(self, msg, robot_id):
        """
        更新当前物品列表
        """
        with self.lock:
            if not msg.data:
                #self.get_logger().info(f"Robot {robot_id} provided an empty item list.")
                self.items.clear()
                return

            for item in msg.data:
                if all(not self.is_duplicate(item, existing_item) for existing_item in self.items):
                    self.items.append(item)

            #self.get_logger().info(f"Updated item list from robot {robot_id}. Total items: {len(self.items)}")

    def is_duplicate(self, item1, item2):
        """
        判断两个物品是否重复
        """
        return (
            item1.x == item2.x and
            item1.y == item2.y and
            item1.diameter == item2.diameter and
            item1.colour == item2.colour
        )

    def convert_item_to_push_item(self, item):
        """
        将 Item 转换为 PushItem
        """
        push_item = PushItem()
        push_item.id = f"{item.x}-{item.y}-{item.diameter}-{item.colour}"
        push_item.x = item.x
        push_item.y = item.y
        push_item.diameter = item.diameter
        push_item.colour = item.colour
        push_item.value = item.value
        return push_item

    def assign_items_to_robots(self):
        """
        为每个机器人分配最近的物品（基于距离和直径值）
        """
        with self.lock:
            if not self.items:
                self.get_logger().info("No items available for assignment.")
                return

            for robot_id, robot_position in self.robot_positions.items():
                closest_item = None
                min_weighted_distance = float('inf')

                # 遍历物品列表，寻找最优物品
                for item in self.items:
                    distance = self.calculate_distance(robot_position, (item.x, item.y))
                    weighted_distance = distance / (item.diameter + 1e-6)  # 防止除以零

                    if weighted_distance < min_weighted_distance:
                        min_weighted_distance = weighted_distance
                        closest_item = item

                if closest_item:
                    # 转换为 PushItem 并发布
                    push_item = self.convert_item_to_push_item(closest_item)
                    push_item_list = PushItemList()
                    push_item_list.data.append(push_item)

                    if robot_id in self.robot_publishers:
                        self.robot_publishers[robot_id].publish(push_item_list)
                        self.items.remove(closest_item)  # 从列表中移除已分配的物品
                        self.get_logger().info(
                            f"Assigned item {push_item.id} (diameter={push_item.diameter}) to robot {robot_id}."
                        )
                else:
                    self.get_logger().info(f"No suitable items for robot {robot_id}.")

    def calculate_distance(self, pos1, pos2):
        """
        计算两点之间的欧几里得距离
        """
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def main(args=None):
    rclpy.init(args=args)

    node = ItemManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
