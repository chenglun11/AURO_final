import sys
import rclpy
import uuid
from rclpy.node import Node
from rclpy.qos import QoSProfile
from assessment_interfaces.msg import Item, ItemList
from solution_interfaces.msg import PushItem, PushItemList
import hashlib
from threading import Lock

class ItemManager(Node):
    def __init__(self):
        super().__init__('item_manager')

        # 订阅器和发布器
        self.item_upstream_subscription = self.create_subscription(
            ItemList,
            '/robot1/items',
            self.item_callback,
            QoSProfile(depth=10)
        )
        self.item_publisher = self.create_publisher(
            PushItemList,
            '/fixed_items',
            QoSProfile(depth=10)
        )
        self.item_activation_subscription = self.create_subscription(
            PushItem,
            '/item_activate',
            self.item_activation_callback,
            QoSProfile(depth=10)
        )

        # 本地物品存储
        self.items = []  # 当前物品列表
        self.assigned_items = {}  # 已分配的物品 (key: item.id, value: PushItem)
        self.requests = []  # 激活请求队列
        self.lock = Lock()  # 用于防止竞争条件的锁

    def generate_unique_id(self, item):
        """
        基于物品的属性生成唯一标识符
        """
        data = f"{item.x}-{item.y}-{item.diameter}-{item.colour}"
        return hashlib.md5(data.encode()).hexdigest()

    def item_callback(self, msg):
        """
        接收物品列表并更新内部存储
        """
        updated_items = []

        for upstream_item in msg.data:
            # 生成唯一 ID
            unique_id = self.generate_unique_id(upstream_item)

            if unique_id in self.assigned_items:
                updated_items.append(self.assigned_items[unique_id])
                ##self.get_logger().info(f"Retaining assigned item: {unique_id}")
            else:
                item = PushItem()
                item.id = unique_id
                item.x = upstream_item.x
                item.y = upstream_item.y
                item.diameter = upstream_item.diameter
                item.colour = upstream_item.colour
                item.value = upstream_item.value
                item.status = 'free'

                updated_items.append(item)

        self.items = updated_items
        self.publish_items()

    def item_activation_callback(self, msg):
        """
        激活物品并标记为已分配
        """
        with self.lock:  # 确保线程安全
            if msg.id in self.assigned_items:
                self.get_logger().warning(f"Item {msg.id} already assigned. Ignoring activation request.")
                return  # 直接退出

            for item in self.items:
                if item.id == msg.id:
                    if item.status == 'free':
                        item.status = 'assigned'
                        self.assigned_items[item.id] = item  # 将物品存入已分配字典
                        self.items.remove(item)  # 从当前物品列表移除
                        self.get_logger().info(f"Item {item.id} assigned and removed from active list.")
                    else:
                        self.get_logger().warning(f"Item {item.id} already assigned. Ignoring activation request.")
                    break
            else:
                self.get_logger().warning(f"Item with ID {msg.id} not found.")

            # 重新发布物品列表
            self.publish_items()

    def publish_items(self):
        """
        发布当前未被分配的物品
        """
        item_list_msg = PushItemList()
        item_list_msg.data = [item for item in self.items if item.status == 'free']
        self.item_publisher.publish(item_list_msg)
        #self.get_logger().info(f"Published {len(item_list_msg.data)} free items.")

    def resolve_requests(self):
        """
        按请求队列处理物品激活请求
        """
        while self.requests:
            request = self.requests.pop(0)
            self.item_activation_callback(request)

    def add_request(self, msg):
        """
        添加激活请求到队列并尝试处理
        """
        self.requests.append(msg)
        #self.get_logger().info(f"Added activation request for item {msg.id} to queue.")
        self.resolve_requests()


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
