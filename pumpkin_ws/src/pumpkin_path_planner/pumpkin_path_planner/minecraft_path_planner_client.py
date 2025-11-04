import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from pumpkin_msgs.srv import PlanMotion
import math
from pumpkin_path_planner.export_utility import ExportUtility
from PIL import Image

class PumpkinPathPlannerClient(Node):
    def __init__(self):
        # Proper ROS 2 Node initialization with a name
        super().__init__('minecraft_path_planner_client')

        # Define the pumpkin face center (assumes you already have this)
        self.pumpkin_face_center = Pose()
        self.pumpkin_face_center.position.x = 0.0
        self.pumpkin_face_center.position.y = 0.0
        self.pumpkin_face_center.position.z = 0.0
        self.pumpkin_face_center.orientation.x = 0.0
        self.pumpkin_face_center.orientation.y = 0.0
        self.pumpkin_face_center.orientation.z = 0.0
        self.pumpkin_face_center.orientation.w = 1.0

    def generate_texture_path_from_image(self, image_path, pixel_size=0.0125, face_size=0.20, threshold=60):
        """
        Generate carving path from a pixel image (black pixels are carved).

        Args:
            image_path: Path to the image file.
            pixel_size: Size of each pixel in meters (1.25 cm).
            face_size: Size of pumpkin face in meters (assumed square).
            threshold: Intensity threshold to consider pixel "black" (0-255).

        Returns:
            List of PoseArray objects (each continuous carved path is one PoseArray)
        """
        img = Image.open(image_path).convert('L')  # grayscale
        width, height = img.size
        assert width == height, "Image should be square"
        pixels = img.load()

        carved_paths = []
        
        path = PoseArray()
        path.header.frame_id = "pumpkin_face"

        for y in range(height):
            for x in range(width):
                if pixels[x, y] < threshold:  # black pixel
                    pose = Pose()
                    # Convert pixel to position in meters
                    pose.position.x = -face_size/2 + (x + 0.5) * pixel_size
                    pose.position.y = -face_size/2 + (y + 0.5) * pixel_size
                    pose.position.z = self.pumpkin_face_center.position.z
                    pose.orientation = self.pumpkin_face_center.orientation
                    path.poses.append(pose)

        carved_paths.append(path)
        return carved_paths

    def run(self):
        self.get_logger().info('Starting pumpkin carving path planning...')

        tool_paths = self.generate_texture_path_from_image(
            "/mnt/data/db4ea2f3-fc90-4ee8-b939-d066fcde3eef.png",
            pixel_size=0.0125,
            face_size=0.20,
            threshold=60
        )

        self.get_logger().info(f'Generated carving path from texture with {len(tool_paths[0].poses)} poses')

        ExportUtility("pumpkin_carving_tool_path").write_pose_array_list_to_yaml(tool_paths)

        # Example: send to motion planner if needed
        # response = self.send_motion_plan_request(tool_paths)


def main(args=None):
    rclpy.init(args=args)
    planner_client = PumpkinPathPlannerClient()
    try:
        planner_client.run()
    except KeyboardInterrupt:
        pass
    planner_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
