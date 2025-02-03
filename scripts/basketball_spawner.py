#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from std_srvs.srv import Trigger  # Simple Trigger service for RQT tools
import os

class BasketballSpawnerNode(Node):
    def __init__(self):
        super().__init__('basketball_spawner')
        self.get_logger().info('BasketballSpawnerNode initialized.')
        self.spawn_service_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.trigger_service = self.create_service(Trigger, 'spawn_basketball', self.handle_spawn_request)

        while not self.spawn_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        self.get_logger().info('BasketballSpawnerNode ready. Use RQT to trigger spawning.')

    def handle_spawn_request(self, request, response):
        urdf_path = os.path.expanduser('~/ros2_ws/src/hyphen_robot_description/urdf/basketball.urdf')
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF file not found at {urdf_path}")
            response.success = False
            response.message = "URDF file not found"
            return response

        # Set spawn parameters
        spawn_request = SpawnEntity.Request()
        spawn_request.name = "basketball"
        spawn_request.xml = open(urdf_path, 'r').read()
        spawn_request.robot_namespace = ''
        spawn_request.initial_pose.position.x = 0.0
        spawn_request.initial_pose.position.y = 0.0
        spawn_request.initial_pose.position.z = 1.0  # Spawn above the ground

        # Call the spawn service
        future = self.spawn_service_client.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Basketball spawned successfully.")
            response.success = True
            response.message = "Basketball spawned successfully"
        else:
            self.get_logger().error(f"Failed to spawn basketball: {future.exception()}")
            response.success = False
            response.message = f"Failed to spawn basketball: {future.exception()}"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = BasketballSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
