import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from pumpkin_msgs.srv import PlanMotion
import math
from pumpkin_path_planner.export_utility import ExportUtility


class PumpkinPathPlannerClient(Node):

    def __init__(self):
        super().__init__('pumpkin_path_planner_client')
        
        # Create client for motion planning service
        self.motion_plan_client = self.create_client(
            PlanMotion, 
            '/generate_motion_plan'
        )
        
        # Wait for service to be available
        # while not self.motion_plan_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for motion planning service...')
        
        self.get_logger().info('Motion planning service is available!')
        
        # Pumpkin configuration - tool starts at center of carving area
        self.pumpkin_face_center = Pose()
        self.pumpkin_face_center.position.x = 0.0  # Tool already at center
        self.pumpkin_face_center.position.y = 0.0
        self.pumpkin_face_center.position.z = 0.0
        
        # Orientation: z-axis points into the pumpkin
        self.pumpkin_face_center.orientation.x = 0.0
        self.pumpkin_face_center.orientation.y = 0.0
        self.pumpkin_face_center.orientation.z = 0.0
        self.pumpkin_face_center.orientation.w = 1.0
        
        self.pumpkin_radius = 0.15  # meters (approximate radius of carving area)

    def generate_triangle_eyes_path(self):
        """
        Generate path for two triangular eyes.
        
        Returns:
            List of PoseArrays (one for each eye)
        """
        eyes = []
        
        # Left eye
        left_eye = PoseArray()
        left_eye.header.frame_id = "pumpkin_face"
        eye_size = 0.03  # 4cm triangles
        left_center_x = -0.06  # 5cm to the left of center
        left_center_y = -0.06   # 5cm above center
        
        # Triangle vertices (relative to pumpkin center)
        triangle_points = [
            (0, -eye_size),           # top
            (-eye_size, eye_size),  # bottom left
            (eye_size, eye_size),   # bottom right
            (0, -eye_size)            # back to top to close
        ]
        
        for offset_x, offset_y in triangle_points:
            pose = Pose()
            pose.position.x = self.pumpkin_face_center.position.x + left_center_x + offset_x
            pose.position.y = self.pumpkin_face_center.position.y + left_center_y + offset_y
            pose.position.z = self.pumpkin_face_center.position.z
            pose.orientation = self.pumpkin_face_center.orientation
            left_eye.poses.append(pose)
        
        eyes.append(left_eye)
        
        # Right eye (mirror the left)
        right_eye = PoseArray()
        right_eye.header.frame_id = "pumpkin_face"

        right_center_x = 0.06  # 5cm to the right of center
        
        for offset_x, offset_y in triangle_points:
            pose = Pose()
            pose.position.x = self.pumpkin_face_center.position.x + right_center_x + offset_x
            pose.position.y = self.pumpkin_face_center.position.y + left_center_y + offset_y
            pose.position.z = self.pumpkin_face_center.position.z
            pose.orientation = self.pumpkin_face_center.orientation
            right_eye.poses.append(pose)
        
        eyes.append(right_eye)
        
        return eyes

    def generate_semicircle_mouth_path(self, num_points=24, width_factor=2.0):
        """
        Generate path for a semicircular smiling mouth.
        
        Args:
            num_points: Number of points in the semicircle
            
        Returns:
            PoseArray with poses for the mouth
        """
        mouth = PoseArray()
        mouth.header.frame_id = "pumpkin_face"

        
        mouth_radius = 0.04  # 3.5cm radius for the mouth
        mouth_center_x = 0.0   # Centered horizontally
        mouth_center_y = 0.06 # 5cm below center
        
        # Create semicircle (bottom half, smiling)
        # Angle goes from 0 to pi (180 degrees)
        for i in range(num_points + 1):
            angle = (i / num_points) * math.pi  # Only pi, not 2*pi for semicircle

            pose = Pose()
            
            # Calculate position on semicircle
            # Starting from right side, curving down and ending at left side
            x_offset = mouth_radius * math.cos(angle) * width_factor
            y_offset = mouth_radius * math.sin(angle)  # Negative to curve downward (smile)
            
            pose.position.x = self.pumpkin_face_center.position.x + mouth_center_x + x_offset
            pose.position.y = self.pumpkin_face_center.position.y + mouth_center_y + y_offset
            pose.position.z = self.pumpkin_face_center.position.z
            
            # Keep orientation same as pumpkin face
            pose.orientation = self.pumpkin_face_center.orientation
            
            mouth.poses.append(pose)
        #close the mouth loop
        mouth.poses.append(mouth.poses[0])
        
        return mouth

    def generate_complete_face_path(self):
        """
        Generate complete face with two triangle eyes and a semicircular mouth.
        
        Returns:
            List of PoseArrays (two eyes + one mouth)
        """
        paths = []
        
        # Add triangle eyes
        eye_paths = self.generate_triangle_eyes_path()
        paths.extend(eye_paths)
        
        # Add semicircular mouth
        mouth_path = self.generate_semicircle_mouth_path()
        paths.append(mouth_path)
        
        return paths

    def send_motion_plan_request(self, pose_arrays):
        """
        Send the path planning request to the motion planning service.
        
        Args:
            pose_arrays: List of PoseArray objects to carve
            
        Returns:
            Service response with the joint trajectory
        """
        # Create service request
        request = PlanMotion.Request()
        request.path = pose_arrays
        
        self.get_logger().info(f'Sending motion plan request with {len(pose_arrays)} path(s)...')
        
        # Call service asynchronously
        future = self.motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Motion plan received successfully!')
            return future.result()
        else:
            self.get_logger().error('Motion planning service call failed!')
            return None

    def run(self):
        """
        Main execution function - generates path and requests motion plan.
        """
        self.get_logger().info('Starting pumpkin carving path planning...')
        
        # Generate complete face: two triangle eyes and semicircular mouth
        tool_paths = self.generate_complete_face_path()
        
        self.get_logger().info(f'Generated {len(tool_paths)} tool path(s): 2 eyes + 1 semicircular mouth')
        
        ExportUtility("pumpkin_carving_tool_path").write_pose_array_list_to_yaml(tool_paths)

        # # Step 2: Send to motion planner
        # response = self.send_motion_plan_request(tool_paths)
        
        
        
        # Step 3: Save trajectory if successful
        # if response and response.trajectory:
        #     trajectory_msg = response.trajectory  # Get trajectory from service response
        #     ExportUtility("pumpkin_carving").write_joint_trajectory_to_yaml(trajectory_msg)
        #     self.get_logger().info('Pumpkin carving plan complete!')
        #     self.get_logger().info('Trajectory saved to ~/.aims/ directory')
        # else:
        #     self.get_logger().error('Failed to generate motion plan')


def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the client
    planner_client = PumpkinPathPlannerClient()
    
    try:
        planner_client.run()
    except KeyboardInterrupt:
        pass
    
    planner_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()