import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        # 创建 NavigateToPose 的 Action 客户端
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta=0.0):
        # 创建一个 PoseStamped 消息来设置目标位置
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'  # 设置坐标系
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        # 设置朝向
        goal_msg.pose.orientation.z = theta
        goal_msg.pose.orientation.w = 1.0

        # 创建并发送导航目标
        goal = NavigateToPose.Goal()
        goal.pose = goal_msg
        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current progress: {feedback.current_pose}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached successfully')
        else:
            self.get_logger().info('Failed to reach goal')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigator = NavigateToPoseClient()
    navigator.send_goal(2.0, 2.0, 0.0)  # 发送到(2.0, 2.0)位置
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
