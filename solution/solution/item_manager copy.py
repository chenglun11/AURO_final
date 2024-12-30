import sys
import rclpy
import hashlib
from rclpy.node import Node
from rclpy.qos import QoSProfile
from threading import Lock
from assessment_interfaces.msg import Item, ItemList
from solution_interfaces.msg import PushItem, PushItemList

class ItemManager(Node):
    def __init__(self):
        super().__init__('item_manager')
        self.robot_count = self.declare_parameter('robot_count', 2).get_parameter_value().integer_value

        self.global_items = {}  # 全局物品存储 {item_id: PushItem}
        self.assigned_items = {}  # 已分配的物品 {item_id: PushItem}
        self.robot_publishers = {}  # 发布器 {robot_namespace: publisher}
        self.lock = Lock()  # 确保线程安全
        self.timer_period = 1.0  # 每秒发布一次物品列表

        self.item_activation_subscription = self.create_subscription(
            PushItem,
            '/item_activate',
            self.item_activation_callback,
            QoSProfile(depth=10)
        )

        # 初始化每个机器人的订阅和发布
        for robot_id in range(1, self.robot_count + 1):
            self.setup_robot(robot_id)

        # 定时发布任务
        self.create_timer(self.timer_period, self.publish_all_items)

    def setup_robot(self, robot_id):
        robot_namespace = f'robot{robot_id}'

        # 订阅物品数据
        self.create_subscription(
            ItemList,
            f'/{robot_namespace}/items',
            lambda msg, rid=robot_id: self.item_callback(msg, rid),
            QoSProfile(depth=10)
        )

        # 创建发布器
        item_publisher = self.create_publisher(
            PushItemList,
            f'/{robot_namespace}/fixed_items',
            QoSProfile(depth=10)
        )

        self.robot_publishers[robot_namespace] = item_publisher

    def generate_unique_id(self, item, robot_id):
        """
        基于物品的属性生成唯一标识符
        """
        data = f"{robot_id}-{item.x}-{item.y}-{item.diameter}-{item.colour}"
        return hashlib.md5(data.encode()).hexdigest()

    def item_callback(self, msg, robot_id):
        """
        接收物品列表并更新内部存储
        """
        updated = False
        robot_namespace = f'robot{robot_id}'

        for upstream_item in msg.data:
            unique_id = self.generate_unique_id(upstream_item, robot_id)

            with self.lock:
                if unique_id in self.assigned_items:
                    continue

                if unique_id not in self.global_items:
                    item = PushItem()
                    item.id = unique_id
                    item.x = upstream_item.x
                    item.y = upstream_item.y
                    item.diameter = upstream_item.diameter
                    item.colour = upstream_item.colour
                    item.value = upstream_item.value
                    item.status = 'free'
                    item.robot_id = robot_id

                    self.global_items[unique_id] = item
                    updated = True
                    self.get_logger().info(f"Added new item: {unique_id} from {robot_namespace}")

        if updated:
            self.publish_items(robot_namespace)

    def item_activation_callback(self, msg):
        """
        激活物品并标记为已分配
        """
        with self.lock:
            if msg.id in self.assigned_items:
                self.get_logger().warning(f"Item {msg.id} already assigned. Ignoring activation request.")
                return

            if msg.id in self.global_items:
                item = self.global_items[msg.id]
                if item.status == 'free':
                    item.status = 'assigned'                
                    self.publish_all_items()  # 广播状态，通知其他机器人
                    rclpy.spin_once(self, timeout_sec=0.1)  # 等待状态传播
                    self.assigned_items[msg.id] = item
                    self.get_logger().info(f"Item {msg.id} assigned successfully.")
                else:
                    self.get_logger().warning(f"Item {msg.id} is not free.")
            else:
                self.get_logger().warning(f"Item {msg.id} not found.")

    def publish_items(self, robot_namespace):
        """
        发布当前未被分配的物品给指定机器人
        """
        if robot_namespace not in self.robot_publishers:
            self.get_logger().error(f"No publisher found for {robot_namespace}.")
            return

        robot_id = int(robot_namespace.replace('robot', ''))
        with self.lock:
            push_item_list = PushItemList()
            push_item_list.data = [
                item for item in self.global_items.values()
                if item.status == 'free' and item.robot_id == robot_id
            ]

        publisher = self.robot_publishers[robot_namespace]
        if push_item_list.data:
            publisher.publish(push_item_list)
            self.get_logger().info(f"Published {len(push_item_list.data)} items for {robot_namespace}.")
        else:
            self.get_logger().info(f"No items to publish for {robot_namespace}.")

    def publish_all_items(self):
        """
        发布所有物品状态，包括已分配和未分配的物品。
        """
        with self.lock:
            for robot_namespace, publisher in self.robot_publishers.items():
                push_item_list = PushItemList()
                push_item_list.data = list(self.global_items.values())  # 包括所有物品状态
                publisher.publish(push_item_list)
                self.get_logger().info(f"Published {len(push_item_list.data)} items for {robot_namespace}.")


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
