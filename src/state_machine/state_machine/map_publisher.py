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
        super().__init__("map_publisher")

        # Declare parameters so you can change map_name from command line
        self.declare_parameter("map_name", "custom_casa_2")
        self.map_name = self.get_parameter("map_name").value

        # Transient_local ensures RViz gets the map even if it connects AFTER the node starts
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, "/map", latching_qos)

        # Update 'tools' to your actual package name if different
        package_share_dir = get_package_share_directory("state_machine")

        self.map_file_path = os.path.join(
            package_share_dir, "maps", f"{self.map_name}.yaml"
        )

        if not os.path.exists(self.map_file_path):
            self.get_logger().warn(f"Map not found in share: {self.map_file_path}")
            self.map_file_path = f"{self.map_name}.yaml"
            self.get_logger().info(f"Trying local path: {self.map_file_path}")

        try:
            self.map_msg = self.load_map_msg(self.map_file_path)
            self.get_logger().info(f"Map '{self.map_name}' loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            self.map_msg = None

        self.timer = self.create_timer(1.0, self.timer_callback)

    def load_map_msg(self, map_file_path):
        with open(map_file_path, "r") as f:
            map_data = yaml.safe_load(f)

        map_dir = os.path.dirname(map_file_path)
        map_img_path = os.path.join(map_dir, map_data["image"])

        img = Image.open(map_img_path).convert("L")
        img_data = np.array(img)

        occ_th_val = 255 * (1.0 - map_data["occupied_thresh"])

        free_th_val = 255 * (1.0 - map_data["free_thresh"])

        grid = np.full(img_data.shape, -1, dtype=np.int8)

        grid[img_data <= occ_th_val] = 100
        grid[img_data >= free_th_val] = 0

        grid_flipped = np.flipud(grid)
        flat_grid = grid_flipped.flatten().tolist()

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = map_data["resolution"]
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.origin.position.x = map_data["origin"][0]
        msg.info.origin.position.y = map_data["origin"][1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = flat_grid

        return msg

    def timer_callback(self):
        if self.map_msg:
            self.publisher_.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
