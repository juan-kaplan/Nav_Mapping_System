import os
import yaml
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # transient_local ensures new subscribers (RViz) get the last message
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', latching_qos)

        package_share_dir = get_package_share_directory('state_machine')
        map_name = 'custom_casa_1'
        self.map_file_path = os.path.join(package_share_dir, 'maps', f'{map_name}.yaml')
        
        self.map_msg = self.load_map_msg(self.map_file_path)
        
        self.timer = self.create_timer(2.0, self.timer_callback)

    def load_map_msg(self, map_file_path):
        self.get_logger().info(f"Loading map from {map_file_path}")
        
        with open(map_file_path, 'r') as f:
            map_data = yaml.safe_load(f)

        map_dir = os.path.dirname(map_file_path)
        map_img_path = os.path.join(map_dir, map_data['image'])
        
        # 1. Load Image as Grayscale (0-255)
        img = Image.open(map_img_path).convert('L')
        img_data = np.array(img)

        # 2. Calculate Thresholds correctly (based on 0-255 range)
        # occupied_thresh (0.65) -> Pixels darker than this are walls
        # The formula is: threshold_value = 255 * (1 - occupied_thresh)
        occ_th_val = 255 * (1.0 - map_data['occupied_thresh']) 
        
        # free_thresh (0.196) -> Pixels lighter than this are free
        # The formula is: threshold_value = 255 * (1 - free_thresh)
        free_th_val = 255 * (1.0 - map_data['free_thresh'])

        self.get_logger().info(f"Pixel Thresholds -> Wall < {occ_th_val} | Free > {free_th_val}")

        # 3. Create the Grid (Default to -1/Unknown)
        grid = np.full(img_data.shape, -1, dtype=np.int8)

        # 4. Apply Trinary Logic
        # IF pixel is DARKER (lower value) than the occupied threshold -> IT IS A WALL (100)
        grid[img_data <= occ_th_val] = 100
        
        # IF pixel is LIGHTER (higher value) than the free threshold -> IT IS FREE (0)
        grid[img_data >= free_th_val] = 0

        # 5. Flip and Flatten
        grid_flipped = np.flipud(grid)
        flat_grid = grid_flipped.flatten().tolist()

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = map_data['resolution']
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.origin.position.x = map_data['origin'][0]
        msg.info.origin.position.y = map_data['origin'][1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = flat_grid
        
        return msg

    def timer_callback(self):
        if self.map_msg:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()