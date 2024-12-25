import rclpy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros

def main():
    rclpy.init()

    node = rclpy.create_node('tf2_listener')

    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)

    # pose
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "camera"
    pose_stamped.pose.position.x = 1.0
    pose_stamped.pose.position.y = 2.0
    pose_stamped.pose.position.z = 3.0
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    rclpy.spin_once(node, timeout_sec=1)  

    try:
        transform = tf_buffer.lookup_transform('base_link', 'camera', rclpy.time.Time())
        transformed_pose = do_transform_pose_stamped(pose_stamped, transform)
        print(transformed_pose)
    except tf2_ros.LookupException as e:
        print("Transform not available: ", e)
    except tf2_ros.ExtrapolationException as e:
        print("Extrapolation exception: ", e)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
