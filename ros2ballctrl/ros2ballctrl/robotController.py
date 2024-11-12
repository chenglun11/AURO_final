import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from assessment_interfaces.srv import ItemRequest
from assessment_interfaces.msg import ItemList, ZoneList


class robotCOntroller(Node):
    def __init__(self):
        super().__init__('robotController')

        # 服务客户端，用于拾取和放置小球
        self.pick_up_client = self.create_client(ItemRequest, '/pick_up_item')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')

        # 导航客户端，用于路径规划
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 订阅小球和区域信息
        self.item_subscriber = self.create_subscription(ItemList, 'items', self.item_callback, 10)
        self.zone_subscriber = self.create_subscription(ZoneList, 'zone', self.zone_callback, 10)

        # 当前检测到的小球和区域数据
        self.current_items = []
        self.current_zones = []

    def item_callback(self, msg):
        # 获取并保存最新的小球信息
        self.current_items = msg.data

    def zone_callback(self, msg):
        # 获取并保存最新的区域信息
        self.current_zones = msg.data

    def pick_up_item(self, robot_id):
        # 请求服务拾取小球
        request = ItemRequest.Request()
        request.robot_id = robot_id
        future = self.pick_up_client.call_async(request)
        return future

    def offload_item(self, robot_id):
        # 请求服务放置小球
        request = ItemRequest.Request()
        request.robot_id = robot_id
        future = self.offload_client.call_async(request)
        return future

    def navigate_to(self, x, y):
        # 发送导航目标位置
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        self.nav_to_pose_client.send_goal_async(goal_msg)

    def control_loop(self):
        # 控制主循环，处理小球识别、拾取、放置
        if self.current_items:
            for item in self.current_items:
                # 假设当前小球在机器人附近，则拾取
                self.pick_up_item("robot_1")  # 假设只有一个机器人
                self.get_logger().info(f'Picked up item at ({item.x}, {item.y})')

                # 根据小球颜色选择对应的区域放置
                target_zone = next((zone for zone in self.current_zones if zone.zone == item.colour), None)
                if target_zone:
                    self.navigate_to(target_zone.x, target_zone.y)
                    self.offload_item("robot_1")
                    self.get_logger().info(f'Offloaded item at zone ({target_zone.x}, {target_zone.y})')


def main(args=None):
    rclpy.init(args=args)
    robot = robotCOntroller()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()
