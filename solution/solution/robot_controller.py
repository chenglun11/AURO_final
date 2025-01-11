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
from assessment_interfaces.msg import  ItemList
from solution_interfaces.msg import Zone, ZoneList
from rclpy.duration import Duration

from tf_transformations import euler_from_quaternion

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.pick_up_client = self.create_client(ItemRequest, '/pick_up_item')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')

        self.odom_subscriber = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.item_subscriber = self.create_subscription(ItemList, 'items', self.item_callback, 10)
        self.zone_subscriber = self.create_subscription(ZoneList, '/fixed_zones', self.zone_callback, 10)
        self.zone_activation_publisher = self.create_publisher(Zone, '/zone_activate', 10)
        random.seed()
        self.current_items = []
        self.current_zones = []
        self.zone_init_flag = False
        self.spin_action_client = ActionClient(self, Spin, 'spin')
        self.declare_parameter('x', 0.0)
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

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def item_callback(self,msg):
        #self.get_logger().info(f"Received {len(msg.data)} items from ItemSensor.")
        self.current_items = msg.data 

    def zone_callback(self,msg):
        # self.get_logger().info(f"Received {len(msg.data)} items from ZoneSensor.")
        self.current_zones = {
            zone.name: {
                'wx': zone.wx,
                'wy': zone.wy,
                'ww': zone.ww,
                'colour': zone.fixed_colour,
                'status': zone.status
            } for zone in msg.zones
        }

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
        nav to setting pose (x, y, z)。
        """
        # set goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal['wx']
        goal_pose.pose.position.y = goal['wy']
        goal_pose.pose.position.z = 1.0
        goal_pose.pose.orientation.w = goal['ww']
        
        self.get_logger().info(f"\n----------------x = {goal['wx']}, y = {goal['wy']}--------------\n")
        # create requsst
        self.navigator.goToPose(goal_pose)


        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # self.navigator.get_logger().info(
            #     f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
            # automatic cancel if outtime
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.navigator.cancelTask()
        # result 
        result = self.navigator.getResult()
        return result
        # self.item_pickup()
        
    def find_ball_position(self, ball):

        current_target = {}
        estimated_distance = (69.0 * (float(ball.diameter) ** -0.89)) + 0.1 #aims further then nessessary
        ball_x_mult = (0.003 * estimated_distance) + 0.085 #since camera is mounted near front mult changes with distance
        angle_diff = -ball_x_mult * ball.x
        # estimated_angle = self.yaw + angle_diff
        estimated_angle = (self.yaw + angle_diff) % 360  
            
        x = estimated_distance * math.cos(math.radians(estimated_angle)) 
        y = estimated_distance * -math.sin(math.radians(estimated_angle)) 

        current_target['colour'] = ball.colour
        current_target['wx'] = x + (self.pos_x)
        current_target['wy'] = y + self.pos_y
        current_target['ww'] = estimated_angle
        self.get_logger().info(f"Item.x{x:2f}, Item.y{y:2f},Item.w{estimated_angle:2f}")
        self.get_logger().info(f"currect_item:{current_target}")
        return current_target


    def find_closest_item(self):
        if not self.current_items:
            self.get_logger().info("No items available to find the closest one.")
            return None

        min_distance = float('inf')
        closest_item = None

        for item in self.current_items:
            # cacu the distance of the item and the robot
            distance = math.sqrt(item.x ** 2 + item.y ** 2)
            self.get_logger().info(
                f"Item - Colour: {item.colour}, Distance: {distance:.2f}"
            )
            # update the min_distance and the cloest items
            if distance < min_distance:
                min_distance = distance
                closest_item = item

        self.get_logger().info(
            f"Closest item: {closest_item}"
        )
        return closest_item

    def nav_to_ball(self,current_target):
        """ 
        nav to ball branch
        """
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

        target_colour = current_target['colour'] 
        #check colour
        zone_name, zone_data  = self.check_zone_available(target_colour)

        if not zone_name:  # no zones
            self.get_logger().error(f"No available zone for colour: {target_colour}")
            return False

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
        
    def enact_bot(self):
            self.loop += 1
            self.get_logger().info(f"第 {self.loop} 个循环")
            # received the goal pose
            finded_target = self.find_ball_position(self.find_closest_item())
            # Transit navigation
            self.nav_loop(finded_target)
            self.get_logger().info(f"清空中，正在初始化下次循环")
            self.cleanup()

    def cleanup(self):
        self.nav_to_ball_flag = False
        self.nav_to_zone_flag = False
        self.current_items = []

    def spin(self):
        """
        use spin_until_future_complete to random angle spin the robot
        """
        if not self.spin_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Spin action server not available!")
            return False

        target_yaw = random.uniform(1.0,4.0)
        goal = Spin.Goal()
        goal.target_yaw = target_yaw

        self.get_logger().info(f"Sending spin goal: {target_yaw:.2f} radians")
        goal_future = self.spin_action_client.send_goal_async(goal)

        # wait goal_future 
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)  
        if not goal_future.done():
            self.get_logger().error("Failed to send spin goal: Future not done.")
            return False

        # get goal_handle
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Spin goal rejected or goal_handle is None!")
            return False

        self.get_logger().info("Spin goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()

        # wait result_future finish
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)  
        if not result_future.done():
            self.get_logger().error("Failed to get spin result: Future not done.")
            return False

        # receive results
        try:
            result = result_future.result()
            if result is None:
                self.get_logger().error("Spin result is None!")
                return False
            self.get_logger().info(f"Spin completed successfully in {result.total_elapsed_time.sec} seconds.")
            return True
        except Exception as e:
            self.get_logger().error(f"Spin failed: {e}")
            return False

    def find_items(self):
        """
        check current_items is/not null ，if null -> spin robot。
        """
        if not self.current_items:
            self.get_logger().info("current_items is empty. Attempting to spin...")

            #  spin()
            spin_result = self.spin()
            self.get_logger().info(f"Spin result: {spin_result}")
            if spin_result:
                self.get_logger().info("Spin completed successfully. Checking for new items...")
                if self.current_items:  # check the new items
                    self.get_logger().info("New items found after spin.")
                    return True
                else:
                    self.get_logger().warning("No items found even after spin.")
                    return False
            else:
                self.get_logger().error("Spin failed. Unable to proceed.")
                return False

        self.get_logger().info("current_items is not empty. Processing item...")
        return True


    def control_loop(self):
        """
        main controller loop, conbine find_items and enact_bot logicl
        """
        if not self.set_init_pose_flag:
            self.initial_robot_pose()
            return

        # call find_item, insure the valid items
        if not self.find_items():
            self.get_logger().warning("No items to process. Waiting for next iteration...")
            return  # wait next loop

        if self.loop > 0:
            self.get_logger().info(f"Starting navigation loop {self.loop}.")

            # call enact_bot function
            if not self.enact_bot():
                self.get_logger().error("Enact bot failed. Skipping to next iteration.")
                return  # exit loop

            # wait next item
            self.get_logger().info("Waiting for next Item...")
        else:
            self.get_logger().info("No loops remaining. Exiting control loop.")

    def check_zone_available(self, colour):
        """
        Checks to see if there is a region that has been assigned the specified color, and if not, activates a free region.
        """

        shuffled_zones = list(self.current_zones.items())
        random.shuffle(shuffled_zones)
        self.get_logger().info(f"random zone is {shuffled_zones}")
        # First find the area that has been assigned to the specified color
        for zone_name, zone_data in shuffled_zones:
            #self.get_logger().info(f"All zones :{zone_name},:{zone_data}1(type: {type(zone_data['colour'])}")
            if zone_data['colour'].strip().lower() == colour.strip().lower():
                self.get_logger().info(f"Zone {zone_name} already assigned to colour {colour}")
                return zone_name, zone_data  # Returns the name of the assigned area

        # Finding areas with unassigned colors
        for zone_name, zone_data in shuffled_zones:
            #self.get_logger().info(f"All zones :{zone_name}, - colour: {repr(zone_data['colour'])}:{zone_data}2")
            if zone_data['colour'].strip().lower() == 'none':
                self.get_logger().info(f"Activating zone {zone_name} for colour {colour}")
                self.activate_zone(zone_name, colour)  # Activate zones
                return zone_name, zone_data 

        # If there are no available areas
        self.get_logger().error(f"No available zones for colour {colour}.")
        return None,None

    def activate_zone(self, zone_name, colour):
        """
        Release of regional activation information
        """
        if zone_name not in self.current_zones:
            self.get_logger().error(f"Zone {zone_name} does not exist!")
            return

        zone_data = self.current_zones[zone_name]
        zone_msg = Zone(
            name=zone_name,
            wx=zone_data['wx'],
            wy=zone_data['wy'],
            ww=zone_data['ww'],
            fixed_colour=colour,
            status='assigned'
        )
        self.current_zones = {}
        self.zone_activation_publisher.publish(zone_msg)
        self.get_logger().info(f"Activated zone: {zone_name} for colour {colour}")
        
    def destroy_node(self):
        self.get_logger().info("Shutting down spin action server...")
    # Ensure that all targets are correctly canceled
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