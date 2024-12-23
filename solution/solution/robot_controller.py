import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan,CameraInfo
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ExtrapolationException
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from auro_interfaces.srv import ItemRequest
from assessment_interfaces.msg import ItemList, ZoneList
from rclpy.duration import Duration
import numpy as np

class RobotController(Node):


    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value
        
        self.pick_up_client = self.create_client(ItemRequest, '/pick_up_item')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')

        self.item_subscriber = self.create_subscription(ItemList, 'items', self.item_callback, 10)
        self.zone_subscriber = self.create_subscription(ZoneList, 'zone', self.zone_callback, 10)
        # self.lidar_subscriber = self.create_subscription(LaserScan,'scan',self.scan_callback,10)
        self.camera_subscriber = self.create_subscription(CameraInfo,'/robot1/camera/camera_info',self.camera_info_callback,10)
        # self.image_subscriber = self.create_subscription(Image,'/robot1/camera/image_raw',self.image_callback,10)
        self.current_items = []
        self.current_zones = []

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.set_init_pose_flag = False
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_matrix = None



        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def scan_callback(self,msg):
        """
        处理激光扫描数据，筛选有效方向并计算目标位置
        """
        try:
            # 打印激光扫描的角度范围
            #self.get_logger().info(f"Laser scan angle range: {np.degrees(msg.angle_min)}° to {np.degrees(msg.angle_max)}°")

            # 假设物体的角度范围
            object_angle_min = 30.0  # 最小角度
            object_angle_max = 40.0  # 最大角度

            # 转换角度范围为索引
            angle_increment = msg.angle_increment
            angle_min = msg.angle_min
            index_min = int((np.radians(object_angle_min) - angle_min) / angle_increment)
            index_max = int((np.radians(object_angle_max) - angle_min) / angle_increment)

            # 确保索引范围有效
            index_min = max(0, index_min)
            index_max = min(len(msg.ranges), index_max)

            # 提取目标范围的距离数据
            ranges = np.array(msg.ranges)
            ranges[ranges == float('inf')] = np.nan  # 将无穷大替换为 NaN
            object_ranges = ranges[index_min:index_max]

            # 打印目标范围的激光距离值
           # self.get_logger().info(f"Target distances (raw): {object_ranges}")

            # 筛选有效值
            valid_object_ranges = object_ranges[~np.isnan(object_ranges)]
            #self.get_logger().info(f"Valid target distances: {valid_object_ranges}")

            # 如果没有有效数据，则跳过处理
            if len(valid_object_ranges) == 0:
                self.get_logger().warning("No valid points detected in the target range.")
                return

            # 计算目标点的平均距离
            target_distance = np.nanmean(valid_object_ranges)

            # 计算目标点在雷达坐标系中的角度
            object_angle = (index_min + index_max) / 2 * angle_increment + angle_min

            # 转换为雷达坐标系下的 (x, y)
            x_lidar = target_distance * np.cos(object_angle)
            y_lidar = target_distance * np.sin(object_angle)

            # 打印目标在雷达坐标系中的位置
            self.get_logger().info(f"Target position in lidar frame: ({x_lidar:.2f}, {y_lidar:.2f})")

            # 构造 PoseStamped
            pose_in_lidar = PoseStamped()
            pose_in_lidar.header.frame_id = 'robot1/base_scan'
            pose_in_lidar.header.stamp = self.get_clock().now().to_msg()
            pose_in_lidar.pose.position.x = x_lidar
            pose_in_lidar.pose.position.y = y_lidar
            pose_in_lidar.pose.position.z = 0.0
            pose_in_lidar.pose.orientation.w = 1.0

            # 转换到世界坐标系
            pose_in_map = self.lidar_transform_pose(pose_in_lidar)

            if pose_in_map:
                wx = pose_in_map.pose.position.x
                wy = pose_in_map.pose.position.y
                wz = pose_in_map.pose.position.z
                self.get_logger().info(f"World coordinates: ({wx:.2f}, {wy:.2f}, {wz:.2f}),Naving")

        except Exception as e:
                self.get_logger().error(f"Error processing scan data: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        """
        获取相机内参矩阵
        """
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        # self.get_logger().info(f"Camera matrix received:\n{self.camera_matrix}")

    def estimate_depth(self, u, v, pixel_diameter, actual_diameter):
        """
        通过小球直径估算深度，并计算相机坐标
        """
        if self.camera_matrix is None:
            self.get_logger().warning("Camera info not received yet.")
            return None

        # 提取相机内参
        fx = self.camera_matrix[0, 0]  # 焦距 (水平)
        fy = self.camera_matrix[1, 1]  # 焦距 (垂直)
        cx = self.camera_matrix[0, 2]  # 主点 (水平)
        cy = self.camera_matrix[1, 2]  # 主点 (垂直)

        # 计算深度 Z
        Z = (fx * actual_diameter) / pixel_diameter
        self.get_logger().info(f"Estimated depth Z: {Z:.2f} meters")

        # 计算相机坐标 (X, Y, Z)
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        self.get_logger().info(f"Camera coordinates: (X: {X:.2f}, Y: {Y:.2f}, Z: {Z:.2f})")

        return X, Y, Z

    def convert_to_world_coordinates(self, x, y, z):
        """
        如果需要将相机坐标 (X, Y, Z) 转换为世界坐标，可以通过 TF 实现。
        """
        pose_in_camera = PoseStamped()
        pose_in_camera.header.frame_id = 'camera_link'
        pose_in_camera.pose.position.x = x
        pose_in_camera.pose.position.y = y
        pose_in_camera.pose.position.z = z
        pose_in_camera.pose.orientation.w = 1.0  # 无旋转

        # TODO: 使用 TF 转换到世界坐标系
        pass

    def item_callback(self,msg):
        # self.get_logger().info(f"Received {len(msg.data)} items from ItemSensor.")
        self.current_items = msg.data  # 存储物品列表

    def zone_callback(self,msg):
        # self.get_logger().info(f"Received {len(msg.data)} items from ZoneSensor.")
        self.current_zones = msg.data


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

    def initial_robot_pose(self,x,y,yaw):
        navigator = BasicNavigator()
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        navigator.setInitialPose(initial_pose)
        self.set_init_pose_flag = True

    def nav_to_pose(self,x,y,z):
        """
              导航到指定位置 (x, y, yaw)。
              """

        # 等待导航动作服务器可用
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return False
        navigator = BasicNavigator()
        # 等待导航启动完成
        navigator.waitUntilNav2Active()

        # 设置目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.w = 1.0

        # 创建导航目标请求
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            navigator.get_logger().info(
                f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
            # 超时自动取消
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()
        # 最终结果判断
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            navigator.get_logger().error('导航结果：失败')
        else:
            navigator.get_logger().error('导航结果：返回状态无效')
        
        return result

    def transform_pose(self,x,y):
      # 构造一个 PoseStamped，以 camera_link 为参考坐标系
        pose_in_camera = PoseStamped()
        pose_in_camera.header.frame_id = 'robot1/camera_link'
        pose_in_camera.header.stamp = self.get_clock().now().to_msg()
        pose_in_camera.pose.position.x = x
        pose_in_camera.pose.position.y = y
        pose_in_camera.pose.position.z = 0.0
        pose_in_camera.pose.orientation.x = 0.0
        pose_in_camera.pose.orientation.y = 0.0
        pose_in_camera.pose.orientation.z = 0.0
        pose_in_camera.pose.orientation.w = 1.0

        target_frame = 'map'
        source_frame = 'camera_link'

        try:
            # 等待 TF 缓冲区中加载完变换
            can_tf = self.tf_buffer.can_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),  # 使用最新可用时间
            timeout=rclpy.duration.Duration(seconds=3.0)  # 等待3秒
            )
            if not can_tf:
                self.get_logger().error(f"Transform from {source_frame} to {target_frame} not available after 3s.")
                return

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            # 使用 do_transform_pose 进行坐标变换
            pose_in_map = do_transform_pose_stamped(pose_in_camera, transform)

            # 输出结果
            wx = pose_in_map.pose.position.x
            wy = pose_in_map.pose.position.y
            wz = pose_in_map.pose.position.z
            

            self.get_logger().info(f"Pose in map: ({wx:.2f}, {wy:.2f}, {wz:.2f}), Waiting to Nav")

            #self.nav_to_pose(wx,wy,wz)
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().error(f"Transform not available: {str(e)}")
            return wx,wy,wz

    def lidar_transform_pose(self,pose_in_lidar:PoseStamped):
        """
        使用 TF 将 PoseStamped 从雷达坐标系转换到世界坐标系
        """
        try:
            # 从 TF 中获取变换
            transform = self.tf_buffer.lookup_transform(
                'map',        # 目标坐标系
                'base_scan', # 雷达坐标系
                rclpy.time.Time()    # 最新时间
            )
            # 执行变换
            pose_in_map = do_transform_pose_stamped(pose_in_lidar, transform)
            wx = pose_in_map.pose.position.x
            wy = pose_in_map.pose.position.y
            wz = pose_in_map.pose.position.z
            self.get_logger().info(f"World coordinates: ({wx:.2f}, {wy:.2f}, {wz:.2f})")
            return pose_in_map
        except Exception as e:
            self.get_logger().error(f"Lidar TF transform error: {e}")
            return None
        
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


    def control_loop(self):

        if not self.set_init_pose_flag:
            self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")
            self.initial_robot_pose(self.initial_x,self.initial_y,self.initial_yaw)
            return 
        
        if not self.current_zones:
            self.get_logger().info("No zone detected.")
            return

        if not self.current_items:
            self.get_logger().info("No items detected.")
            return
        
        cloest_item = self.find_closest_item()
        if cloest_item:
            x,y,z = self.estimate_depth(cloest_item.x,cloest_item.y,cloest_item.diameter,0.1)
            self.nav_to_pose(x,y,z)

        # for item in self.current_items:
        #     self.get_logger().info(
        #         f"{self.robot_id}:Item detected {item} - Colour: {item.colour}, X: {item.x}, Y: {item.y}, Diameter: {item.diameter}, Value: {item.value}"
        #     )
        # x = float(self.current_items[0].x)
        # y = float(self.current_items[0].y)
        # self.transform_pose(x,y)

        #closest_item = self.find_closest_item()
        # self.get_logger().info(f"closet item is {closest_item}")

        # if closest_item:
        #     navflag = self.nav_to_pose(closest_item.x.double_value, closest_item.y.double_value,1.0)
        #     if navflag == TaskResult.SUCCEEDED:
        #         self.get_logger().info(f"Arrive at cloest item")
        #         self.item_pickup()
        #self.nav_to_pose()


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