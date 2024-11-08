import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge
import numpy as np

class ItemDetectionNode(Node):
    def __init__(self):
        super().__init__('item_detection_node')
        self.bridge = CvBridge()

        # 订阅原始图像话题
        self.image_subscriber = self.create_subscription(
            Image, '/robot1/camera/image_items', self.image_callback, 10)

        # 发布带有检测标记的图像
        self.marker_publisher = self.create_publisher(Marker, 'item_markers', 10)

        # 颜色阈值定义（HSV 格式）
        self.color_ranges = {
            'RED': ([0, 100, 100], [10, 255, 255]),
            'GREEN': ([45, 100, 100], [75, 255, 255]),
            'BLUE': ([105, 100, 100], [135, 255, 255]),
            'CYAN_ZONE': ([80, 100, 100], [100, 255, 255]),
            'PURPLE_ZONE': ([140, 100, 100], [160, 255, 255]),
            'PINK_ZONE': ([160, 100, 100], [180, 255, 255]),
            'GREEN_ZONE': ([45, 100, 100], [75, 255, 255])
        }

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        markers = []

        for color, (lower, upper) in self.color_ranges.items():
            lower_bound = np.array(lower, dtype=np.uint8)
            upper_bound = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 500:  # 忽略过小的区域
                    moments = cv2.moments(contour)
                    if moments['m00'] != 0:
                        center_x = int(moments['m10'] / moments['m00'])
                        center_y = int(moments['m01'] / moments['m00'])
                        marker = self.create_marker(center_x, center_y, color)
                        markers.append(marker)

        for marker in markers:
            self.marker_publisher.publish(marker)

    def create_marker(self, x, y, color_name):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = self.get_color_rgba(color_name)
        return marker

    def get_color_rgba(self, color_name):
        color_map = {
            'RED': ColorRGBA(1.0, 0.0, 0.0, 1.0),
            'GREEN': ColorRGBA(0.0, 1.0, 0.0, 1.0),
            'BLUE': ColorRGBA(0.0, 0.0, 1.0, 1.0),
            'CYAN_ZONE': ColorRGBA(0.0, 1.0, 1.0, 1.0),
            'PURPLE_ZONE': ColorRGBA(0.5, 0.0, 0.5, 1.0),
            'PINK_ZONE': ColorRGBA(1.0, 0.0, 1.0, 1.0),
            'GREEN_ZONE': ColorRGBA(0.0, 1.0, 0.0, 1.0)
        }
        return color_map.get(color_name, ColorRGBA(1.0, 1.0, 1.0, 1.0))

def main(args=None):
    rclpy.init(args=args)
    node = ItemDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
