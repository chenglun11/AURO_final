from enum import IntEnum
import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from nav2_msgs.action import Spin 
from tf2_ros import TransformListener, Buffer
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from auro_interfaces.srv import ItemRequest
# from assessment_interfaces.msg import  ZoneList,ItemList
from solution_interfaces import ZoneList
from rclpy.duration import Duration

from tf_transformations import euler_from_quaternion

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.pick_up_client = self.create_client(ItemRequest, '/pick_up_item')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')

        self.odom_subscriber = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.item_subscriber = self.create_subscription(ItemList, 'items', self.item_callback, 10)
        self.zone_subscriber = self.create_subscription(ZoneList, 'zone', self.zone_callback, 10)
        random.seed()
        self.current_items = []
        self.current_zones = []
        self.zone_init_flag = False
        self.spin_action_client = ActionClient(self, Spin, 'spin')
        self.declare_parameter('x', -3.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('robot_id', 'robot1')

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y =  self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.robot_id = self.get_parameter('robot_id').value
        self.set_init_pose_flag = False

        self.navigator = BasicNavigator()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  

        self.loop = 1
        self.item_held = None
        self.nav_to_ball_flag = False
        self.nav_to_zone_flag = False
        self.current_target = {}
        self.pos_x = self.initial_x
        self.pos_y = self.initial_y
        self.yaw =  self.initial_yaw
        self.navigator.waitUntilNav2Active()
        self.get_logger().info(f"\t初始化完成\t")


        # self.zones = {
        #     'TOP_RIGHT': {'c':None,'wx': 2.35, 'wy': -2.5,'ww':1.0},      # 右上角
        #     'TOP_LEFT': {'c':None,'wx': 2.35, 'wy': 2.35,'ww':1.0},     # 左上角
        #     'BUT_LEFT': {'c':None,'wx': -3.25, 'wy': 2.35,'ww':1.0}, # 左下角
        #     'BUT_RIGHT': {'c':None,'wx': -3.25, 'wy': -2.35,'ww':1.0}       # 左下角
        # }

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def item_callback(self,msg):
        #self.get_logger().info(f"Received {len(msg.data)} items from ItemSensor.")
        self.current_items = msg.data # 存储物品列表

    def zone_callback(self,msg):
        # self.get_logger().info(f"Received {len(msg.data)} items from ZoneSensor.")
        self.current_zones = msg.data
        #self.current_zones = {current_zones.name}

    def odom_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w])
        
        self.pos_x = msg.pose.pose.position.x + self.initial_x
        self.pos_y = msg.pose.pose.position.y + self.initial_y
        
        self.yaw = -math.degrees(yaw)
        if self.yaw < 0:
            self.yaw += 360 #normalise to 0-360 to act as a bearing
    
    def item_pickup(self, robot_id):
        if not self.pick_up_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Pick-up service not available.")
            return

        requests = ItemRequest.Request()
        requests.robot_id = robot_id
        log = self.pick_up_client.call_async(requests)
        return log
        
    def item_offload(self, robot_id):
        if not self.offload_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Offload service not available.")
            # return
        requests = ItemRequest.Request()
        requests.robot_id = robot_id
        log = self.offload_client.call_async(requests)
        return log
    
    def initial_robot_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        #initial_pose.header.stamp = navigator.get_clock().now().to_msg()

        initial_pose.header.stamp.sec = 0
        initial_pose.header.stamp.nanosec = 0

        initial_pose.pose.position.x = self.initial_x
        initial_pose.pose.position.y = self.initial_y
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)
        self.set_init_pose_flag = True

    def nav_to_pose(self,goal):
        """
        导航到指定位置 (x, y, z)。
        """
        # 设置目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal['wx']
        goal_pose.pose.position.y = goal['wy']
        goal_pose.pose.position.z = 1.0
        goal_pose.pose.orientation.w = goal['ww']
        
        self.get_logger().info(f"\n----------------x = {goal['wx']}, y = {goal['wy']}--------------\n")
        # 创建导航目标请求
        self.navigator.goToPose(goal_pose)


        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # self.navigator.get_logger().info(
            #     f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
            # 超时自动取消
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.navigator.cancelTask()
        # 最终结果判断
        result = self.navigator.getResult()
        return result
        # self.item_pickup()
        
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

    def nav_to_ball(self,current_target):
        item_result = self.nav_to_pose(current_target)
        if item_result == TaskResult.SUCCEEDED:
            self.nav_to_ball_flag = True
            pick_result = self.item_pickup(self.robot_id)
            if pick_result.result() is not None:
                return True
                #self.get_logger().info(f"Pick-up result: {pick_result.result().success}")
            else:
                self.get_logger().error("Failed to call pick-up service.")
                return False
        else:
            return False
        
    def nav_to_zone(self,current_target):
        #拿颜色
        target_colour = current_target['colour'] 
        #检查颜色是否可以,并返回zone
        checked_zone = self.check_zone_available(target_colour)

        if not checked_zone:  # 如果没有可用的区域
            self.get_logger().error(f"No available zone for colour: {target_colour}")
            return False

        zone_name, zone_data = list(checked_zone.items())[0]  # 拿到第一个被分配的区域
        self.get_logger().info(f"Assigned Zone: {zone_name}, Data: {zone_data}")

        zone_target = zone_data

        self.get_logger().info(f"Zone Pose:{zone_target}")
        zone_result = self.nav_to_pose(zone_target)
        if zone_result == TaskResult.SUCCEEDED:
            self.nav_to_zone_flag = True
            offload_result = self.item_offload(self.robot_id)
            if offload_result.result() is not None:
                return True
                #self.get_logger().info(f"Pick-up result: {offload_result.result().success}")
            else:
                self.get_logger().error("Failed to call offload service.")  
                return False
            
    def nav_loop(self,target):
        self.get_logger().info(f"中转值为：{target}")
        self.nav_to_ball(target)
        self.nav_to_zone(target)
        

    def get_robot_pose_tf2(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            self.pos_x = transform.transform.translation.x
            self.pos_y = transform.transform.translation.y
            self.yaw = transform.transform.rotation.w
            self.get_logger().info(f"TF2 Position: x={self.pos_x}, y={self.pos_y}")
        except Exception as e:
            self.get_logger().error(f"Failed to get robot pose via tf2: {e}")
        
    def enact_bot(self):
            self.loop += 1
            self.get_logger().info(f"第 {self.loop} 个循环")
            self.find_another_target()
            # 接收到返回坐标
            finded_target = self.find_ball_position(self.find_closest_item())
            # 中转导航
            self.nav_loop(finded_target)
            self.get_logger().info(f"清空中，正在初始化下次循环")
            self.cleanup()

    def cleanup(self):
        self.nav_to_ball_flag = False
        self.nav_to_zone_flag = False
        self.current_items = []

    def spin(self):
        """
            使用 spin_once 等待 spin 动作完成
            """
        if not self.spin_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Spin action server not available!")
            return False
        target_yaw = random.uniform(3.0,6.0)
        goal = Spin.Goal()
        goal.target_yaw = target_yaw

        self.get_logger().info(f"Sending spin goal: {target_yaw:.2f} radians")
        goal_future = self.spin_action_client.send_goal_async(goal)
        rclpy.spin_once(self)  # 等待 goal 发送完成

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Spin goal rejected!")
            return False

        self.get_logger().info("Spin goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()

        # 使用 spin_once 等待结果
        while not result_future.done():
            self.get_logger().info("Processing callbacks...")
            rclpy.spin_once(self, timeout_sec=0.1)  # 每次检查回调队列

        # 获取结果
        try:
            result = result_future.result().result
            self.get_logger().info(f"Spin completed successfully in {result.total_elapsed_time.sec} seconds.")
            return True
        except Exception as e:
            self.get_logger().error(f"Spin failed: {e}")
            return False

    def random_test(self): # only for test
        # 创建独立的随机数生成器
        for _ in range(3):
            print("Iteration start")
            checked_zone = self.check_zone_available("RED")
            print(f"Checked zone: {checked_zone}")
            print("-" * 30)

    #TODO
    def find_another_target(self):
        noitem = True
        while noitem:
            if not self.current_items:
                self.get_logger().info("Waiting for items to be detected...,Spining")
                
                target = self.spin()
                if target:
                    return
                else:
                    self.get_logger().info(f"spin error -from FAT")
            else:
                noitem = False
        return

                 
    def check_zone_available(self, colour):
        """
        查找并分配目标颜色的区域。
        如果找到已有颜色的区域，直接返回。
        如果没有找到，则分配目标颜色到第一个未分配的区域。
        最终返回一个包含分配或找到区域的 `checked_zone`。
        """
        checked_zone = {}  # 用于记录被修改的区域
        zone_names = random.sample(list(self.zones.keys()), len(self.zones))  # 随机化顺序

        for zone_name in zone_names:  # 使用随机化后的区域顺序
            zone_data = self.zones[zone_name]

            # 优先查找相同颜色的区域
            if zone_data['c'] == colour:  # 如果找到已有颜色
                checked_zone[zone_name] = zone_data
                self.get_logger().info(f"Found existing zone for colour '{colour}': {zone_name}")
                return checked_zone  # 立即返回

        # 如果没有找到相同颜色，再查找空闲区域
        for zone_name in zone_names:  # 再次遍历区域
            zone_data = self.zones[zone_name]
            if zone_data['c'] is None:  # 如果找到空闲区域
                zone_data['c'] = colour
                checked_zone[zone_name] = zone_data
                self.get_logger().info(f"Assigned colour '{colour}' to zone '{zone_name}'")
                return checked_zone  # 返回分配的区域

        # 如果没有找到可用区域
        self.get_logger().error(f"No available zone for colour '{colour}'.")
        return checked_zone  # 返回空的 `checked_zone`
    
    #on test
    def print_zone(self):
        for zone_name, zone_data in self.zones.items():
            print(f"Zone: {zone_name}")
            for key, value in zone_data.items():
                print(f"  {key}: {value}")
        print("-" * 30)  # 分隔线

    def control_loop(self):
        if not self.set_init_pose_flag:
            self.initial_robot_pose()
            return
        
        while self.loop > 0:
            self.get_logger().info(f"Starting navigation loop {self.loop}.")
            self.enact_bot()
            #self.spin()
            # self.random_test()
            break
        self.get_logger().info("Waiting For next Item")

        
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