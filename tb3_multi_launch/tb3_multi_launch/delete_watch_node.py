import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import String

class DeleteWatchNode(Node):
    def __init__(self):
        super().__init__('gz_delete_watch')

        self.name = self.declare_parameter('name', 'robot').get_parameter_value().string_value        
        self.create_subscription(String, '/deleted_entities', self.entity_cb, qos.qos_profile_system_default)
        
    def entity_cb(self, data: String):
        if data.data == self.name:
            self.get_logger().info(f'entity {data.data} has been deleted, exiting')
            raise SystemExit
            
def main():
    rclpy.init()
    node = DeleteWatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

    rclpy.shutdown()
