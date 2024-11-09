# src/my_nav2_package/my_nav2_package/robot_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from auro_interfaces.msg import ItemList, ZoneList
from .item_manager_client import ItemManagerClient

class RobotController(Node):
    def __init__(self, robot_id):
        super().__init__('robot_controller')
        self.robot_id = robot_id
        self.item_manager = ItemManagerClient()
        self.holding_item = None
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 订阅物品和区域话题
        self.create_subscription(ItemList, f"/{self.robot_id}/items", self.item_callback, 10)
        self.create_subscription(ZoneList, f"/{self.robot_id}/zones", self.zone_callback, 10)

    def item_callback(self, msg):
        for item in msg.items:
            if not self.holding_item and item.color in ["red", "green", "blue"]:
                # 导航到物品位置
                self.navigate_to(item.x, item.y)
                # 试图拾取物品
                success = self.item_manager.pick_up(self.robot_id, item.id)
                if success:
                    self.holding_item = item.color
                    self.get_logger().info(f"Picked up {item.color} item")
                break

    def zone_callback(self, msg):
        if self.holding_item:
            for zone in msg.zones:
                if zone.color == self.holding_item:
                    # 导航到存放区域位置
                    self.navigate_to(zone.x, zone.y)
                    # 放置物品
                    success = self.item_manager.offload(self.robot_id, zone.id)
                    if success:
                        self.get_logger().info(f"Offloaded {self.holding_item} item at {zone.color} zone")
                        self.holding_item = None
                    break

    def navigate_to(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        self.nav_action_client.wait_for_server()
        self.nav_action_client.send_goal_async(nav_goal, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current navigation progress: {feedback.current_pose}")

def main(args=None):
    rclpy.init(args=args)
    robot_id = 'robot1'  # 根据实际情况设定机器人ID
    robot_controller = RobotController(robot_id)
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
