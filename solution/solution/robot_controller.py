import math
import sys

import rclpy
from enum import Enum
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
from auro_interfaces.srv import ItemRequest
from assessment_interfaces.msg import ItemList, ZoneList
from nav_msgs.msg import Odometry


class ZoneLocation(Enum):
    TOP_LEFT = 1
    TOP_RIGHT = 2
    BOTTOM_RIGHT = 3
    BOTTOM_LEFT = 4


class Zone():
    def __init__(self,location, x, y, color):
        self.location = location
        self.x = x
        self.y = y
        self.color = None
class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.pick_up_client = self.create_client(ItemRequest, '/pick_up_item')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')

        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10)
        self.zone_subscriber = self.create_subscription(
            ZoneList,
            'zone',
            self.zone_callback,
            10)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,)

        self.current_items = []
        self.current_zones = []
        self.items_received = False  # 是否收到物品列表
        self.zones_received = False  # 是否收到区域列表

        self.declare_parameter('zone_top_left', True)
        self.declare_parameter('zone_top_right', True)
        self.declare_parameter('zone_bottom_left', True)
        self.declare_parameter('zone_bottom_right', True)

        zone_top_left = self.get_parameter('zone_top_left').value
        zone_top_right = self.get_parameter('zone_top_right').value
        zone_bottom_left = self.get_parameter('zone_bottom_left').value
        zone_bottom_right = self.get_parameter('zone_bottom_right').value

        # Initialize zones
        self.zones = {}
        if zone_top_left:
            self.zones[ZoneLocation.TOP_LEFT] = Zone(ZoneLocation.TOP_LEFT, 2.57, 2.5)
        if zone_top_right:
            self.zones[ZoneLocation.TOP_RIGHT] = Zone(ZoneLocation.TOP_RIGHT, 2.57, -2.46)
        if zone_bottom_right:
            self.zones[ZoneLocation.BOTTOM_RIGHT] = Zone(ZoneLocation.BOTTOM_RIGHT, -3.42, -2.46)
        if zone_bottom_left:
            self.zones[ZoneLocation.BOTTOM_LEFT] = Zone(ZoneLocation.BOTTOM_LEFT, -3.42, 2.5)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.yaw = 0.0
        self.declare_parameter('x', -3.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)


    def item_callback(self,msg):
        self.get_logger().info(f"Received {len(msg.data)} items from ItemSensor.")
        self.current_items = msg.data  # 存储物品列表
        self.items_received = True  # 标志收到物品数据

    def zone_callback(self,msg):
        self.get_logger().info(f"Received {len(msg.data)} zones from ZoneSensor.")
        self.current_zones = msg.data
        self.zones_received = True  # 标志收到区域数据

    def odom_callback(self, msg):
        self.pose = msg.pose.pose  # Store the pose in a class variable

        # Uses tf_transformations package to convert orientation from quaternion to Euler angles (RPY = roll, pitch, yaw)
        # https://github.com/DLu/tf_transformations
        #
        # Roll (rotation around X axis) and pitch (rotation around Y axis) are discarded
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])

        self.yaw = yaw  # Store the yaw in a class variable

    def item_pickup(self, robot_id):
        if not self.pick_up_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Pick-up service not available.")
            return

        requests = ItemRequest.Request()
        requests.robot_id = robot_id
        log = self.pick_up_client.call_async(requests)

        if log.result() is not None:
            self.get_logger().info(f"Pick-up result: {log.result().success}")
        else:
            self.get_logger().error("Failed to call pick-up service.")
        return log

    def item_offload(self, robot_id):
        requests = ItemRequest.Request()
        requests.robot_id = robot_id
        log = self.offload_client.call_async(requests)
        return log

    def nav_to_pose(self,x,y,yaw=0.0):
        """
              导航到指定位置 (x, y, yaw)。
              """
        # 等待导航动作服务器可用
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return False

        x = float(x)
        y = float(y)
        # 设置目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  # 使用地图坐标系
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 创建导航目标请求
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Navigating to pose: x={x}, y={y}, yaw={yaw}")

        # 发送目标并等待完成
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if not future.result().accepted:
            self.get_logger().error("Goal rejected by server!")
            return False

        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.result().status == NavigateToPose.Result.SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
            return True
        else:
            self.get_logger().error("Navigation failed!")
            return False

    # unsure
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
            f"Closest item - Colour: {closest_item.colour}, Distance: {min_distance:.2f}"
        )
        return closest_item

    def on_test(self):
        self.get_logger().info("Testing nav_to_pose directly...")
        success = self.nav_to_pose(10.0, 10.0, 0.0)  # 假设目标位置是 (1.0, 1.0)
        if success:
            self.get_logger().info("Navigation test succeeded.")
        else:
            self.get_logger().error("Navigation test failed.")

    def control_loop(self):

        self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")

        if not self.items_received or not self.zones_received:
            self.get_logger().info("Waiting for items and zones data...")
            return  # 等待下一次循环

        closest_item = self.find_closest_item()
       # self.get_logger().info(f"closet item is {closest_item}")

        # for item in self.current_items:
        #     self.get_logger().info(
        #         f"Item detected - Colour: {item.colour}, X: {item.x}, Y: {item.y}, Diameter: {item.diameter}, Value: {item.value}"
        #     )


        # if self.current_items:
        #     self.get_logger().info(f"navigation to {closest_item.x},{closest_item.y}")
        #     success = self.nav_to_pose(float(closest_item.x), float(closest_item.y))
        #     if success:
        #         self.get_logger().info(f"Navigation success")
        # if closest_item:
        #     success = self.nav_to_pose(self.current_items[0].x, self.current_items[0].y)
        #     if success:
        #         self.get_logger().info(f"Arrived at closest item: {closest_item.colour}")
        #     else:
        #         self.get_logger().error("Failed to navigate to closest item.")



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