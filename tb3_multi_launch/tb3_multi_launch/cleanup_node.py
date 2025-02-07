# see also: https://gist.github.com/driftregion/14f6da05a71a57ef0804b68e17b06de5

import rclpy
from rclpy import qos
from rclpy.node import Node
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from gazebo_msgs.srv import GetModelList, DeleteEntity
from std_srvs.srv import Empty
from std_msgs.msg import String

EXCLUDED_ENTITIES = {
    'ground_plane',
    'turtlebot3_world', 'turtlebot3_house', 'turtlebot3_dqn_world',
    'inner_walls', 'turtlebot3_dqn_obstacle1', 'turtlebot3_dqn_obstacle2', 'turtlebot3_dqn_obstacles', 'obstacles' # dqn obstacles
}

class CleanupNode(Node):
    def __init__(self):
        super().__init__('gz_cleanup_node')

        self.callback_group = ReentrantCallbackGroup()

        self.get_models_srv = self.create_client(GetModelList, '/get_model_list', callback_group=self.callback_group)
        self.delete_entity_srv = self.create_client(DeleteEntity, '/delete_entity', callback_group=self.callback_group)

        self.deleted_entities_pub = self.create_publisher(String, '/deleted_entities', qos.qos_profile_system_default)
        
        self.create_service(Empty, '/clean_simulation', self.clean_cb, callback_group=self.callback_group)
        
    def clean_cb(self, request, response):
        event = Event()
        def done_cb(future):
            nonlocal event
            event.set()
        
        # get models
        if not self.get_models_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('cannot contact /get_model_list')
            return response
        future = self.get_models_srv.call_async(GetModelList.Request()); future.add_done_callback(done_cb)
        event.wait()
        models: list[str] = future.result().model_names
        self.get_logger().info(f'models: {models}')

        # figure out what entities to delete
        entities = set(models) - EXCLUDED_ENTITIES

        # delete entities
        if not self.delete_entity_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('cannot contact /delete_entity')
            return response
        for entity in entities:
            future = self.delete_entity_srv.call_async(DeleteEntity.Request(name=entity)); future.add_done_callback(done_cb)
            event.wait()
            self.deleted_entities_pub.publish(String(data=entity))
            self.get_logger().info(f'deleted entity {entity}')

        return response
            
def main():
    rclpy.init()
    node = CleanupNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

    rclpy.shutdown()
