import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2


class LikelihoodMapPublisher(Node):
    def __init__(self):
        super().__init__('likelihood_map_publisher')
        qos = rclpy.qos.QoSProfile(depth=1)
        qos.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(OccupancyGrid, '/likelihood_map', qos)
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )
    
    def map_callback(self, msg):
        SIGMA = 0.2  # meters

        prob_msg = OccupancyGrid()
        prob_msg.header = msg.header
        prob_msg.info = msg.info

        h,w = msg.info.height, msg.info.width
        grid = np.array(msg.data, dtype=np.int8).reshape((h,w))
        
        occ = (grid == 100)
        unknown = (grid == -1)
        free_or_unk = (~occ).astype(np.uint8)

        dist = cv2.distanceTransform(free_or_unk, cv2.DIST_L2, 3)
        d_metres = dist * msg.info.resolution

        p = np.exp(-(d_metres ** 2) / (2 * SIGMA ** 2)) + 1e-6 
        p[unknown] = np.minimum(p[unknown], 0.05)   

        likelihood = (p * 100).astype(np.uint8)

        prob_msg.data = likelihood.flatten(order = 'C').tolist()

        self.pub.publish(prob_msg)
        self.get_logger().info("Published likelihood map")

def main(args=None):
    rclpy.init(args=args)
    node = LikelihoodMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
