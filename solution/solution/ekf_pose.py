import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped

class EKFPose(Node):
    def __init__(self):
        super().__init__('ekf_pose')

        self.robot_id = self.declare_parameter('robot_id', 'robot1').get_parameter_value().string_value
        namespace = self.robot_id
        # 状态向量 [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        self.covariance = np.eye(3)  # 状态协方差矩阵

        # 过程噪声和观测噪声
        self.process_noise = np.diag([0.1, 0.1, 0.05])  # 过程噪声
        self.observation_noise = np.diag([0.5, 0.5, 0.1])  # 观测噪声

        # 订阅器

        self.odom_subscriber = self.create_subscription(Odometry, f'/{namespace}/odom', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, f'/{namespace}/imu', self.imu_callback, 10)

        # 发布器
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, f'/{namespace}/ekf_pose', 10)

        self.last_time = self.get_clock().now()

    def odom_callback(self, msg):
        """
        里程计数据回调，用于预测
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 时间差，单位为秒
        self.last_time = current_time

        # 提取线速度和角速度
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # 扩展卡尔曼滤波预测步骤
        self.predict(v, omega, dt)

    def imu_callback(self, msg):
        """
        IMU数据回调，用于更新
        """
        # 从IMU中提取角速度
        theta_meas = self.state[2] + msg.angular_velocity.z
        self.update(np.array([self.state[0], self.state[1], theta_meas]))

    def predict(self, v, omega, dt):
        """
        EKF预测步骤
        """
        x, y, theta = self.state

        # 运动模型
        if abs(omega) > 1e-5:
            x_new = x + v / omega * (np.sin(theta + omega * dt) - np.sin(theta))
            y_new = y - v / omega * (np.cos(theta + omega * dt) - np.cos(theta))
        else:
            x_new = x + v * dt * np.cos(theta)
            y_new = y + v * dt * np.sin(theta)

        theta_new = theta + omega * dt
        self.state = np.array([x_new, y_new, theta_new])

        # 雅可比矩阵
        F = np.array([
            [1, 0, -v * dt * np.sin(theta)],
            [0, 1, v * dt * np.cos(theta)],
            [0, 0, 1]
        ])

        # 更新协方差
        self.covariance = F @ self.covariance @ F.T + self.process_noise

    def update(self, observation):
        """
        EKF更新步骤
        """
        H = np.eye(3)  # 观测矩阵
        z = observation  # 观测值
        y = z - H @ self.state  # 残差

        # 卡尔曼增益
        S = H @ self.covariance @ H.T + self.observation_noise
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # 更新状态和协方差
        self.state = self.state + K @ y
        self.covariance = (np.eye(3) - K @ H) @ self.covariance

        # 发布更新后的位姿
        self.publish_pose()

    def publish_pose(self):
        """
        发布融合后的位姿
        """
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = self.state[0]
        pose_msg.pose.pose.position.y = self.state[1]
        pose_msg.pose.pose.orientation.z = np.sin(self.state[2] / 2)
        pose_msg.pose.pose.orientation.w = np.cos(self.state[2] / 2)

        # 扩展 3x3 协方差矩阵到 6x6
        covariance_6x6 = np.zeros((6, 6))
        covariance_6x6[:3, :3] = self.covariance  # 前3x3部分赋值为self.covariance

        # 将 6x6 平铺为长度为 36 的列表
        pose_msg.pose.covariance = covariance_6x6.flatten().tolist()

        self.pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFPose()

    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        pass
    finally:
        ekf_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
