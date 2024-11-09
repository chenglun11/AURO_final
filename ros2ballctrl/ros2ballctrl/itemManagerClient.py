# src/my_nav2_package/my_nav2_package/item_manager_client.py

import rclpy
from rclpy.node import Node
from auro_interfaces.srv import ItemRequest


class ItemManagerClient(Node):
    def __init__(self):
        super().__init__('item_manager_client')
        self.pick_up_client = self.create_client(ItemRequest, '/pick_up_item')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')

        while not self.pick_up_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pick up service...')

        while not self.offload_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for offload service...')

    def pick_up(self, robot_id, item_id):
        request = ItemRequest.Request()
        request.robot_id = robot_id
        request.item_id = item_id
        future = self.pick_up_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def offload(self, robot_id, item_id):
        request = ItemRequest.Request()
        request.robot_id = robot_id
        request.item_id = item_id
        future = self.offload_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
