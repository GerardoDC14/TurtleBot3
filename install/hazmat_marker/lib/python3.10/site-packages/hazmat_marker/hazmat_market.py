import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations

class HazmatMarkerNode(Node):
    def __init__(self):
        super().__init__('hazmat_marker_node')
        self.marker_pub = self.create_publisher(MarkerArray, '/hazmat_markers', 10)
        self.hazmat_sub = self.create_subscription(String, '/hazmat_publisher', self.hazmat_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_position = None
        self.hazmat_detected = False
        self.hazmat_name = "No detections"

    def hazmat_callback(self, msg):
        if msg.data != "No detections":
            self.hazmat_detected = True
            self.hazmat_name = msg.data
        else:
            self.hazmat_detected = False

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose
        if self.hazmat_detected:
            self.add_marker()

    def add_marker(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.current_position.position.x
        marker.pose.position.y = self.current_position.position.y
        marker.pose.position.z = 1.0
        marker.pose.orientation = self.current_position.orientation
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = self.hazmat_name
        marker_array.markers.append(marker)

        # Publish markers
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = HazmatMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
