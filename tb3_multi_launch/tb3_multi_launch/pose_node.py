import rclpy
from rclpy.node import Node
from rclpy import qos

from geometry_msgs.msg import Pose, TransformStamped
from gazebo_msgs.msg import ModelStates

from tf2_ros import TransformBroadcaster

class GazeboPoseNode(Node):
    def __init__(self):
        super().__init__('gz_robot_pose')
        
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(ModelStates, '/gazebo/model_states', self.states_cb, qos.qos_profile_system_default)
        
    def states_cb(self, data: ModelStates):
        poses: dict[str, Pose] = dict(zip(data.name, data.pose))
        pose = poses.get(self.robot_name, None)
        if pose is None: return # no robot pose in here

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.robot_frame

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GazeboPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
