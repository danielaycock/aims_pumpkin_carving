"""
SVG to Pumpkin Path Converter
Converts SVG paths into ROS PoseArray format for pumpkin carving

Requirements:
    pip install svgpathtools numpy

Usage:
    1. Create/download an SVG file with your design
    2. Run: python svg_to_path.py your_design.svg
    3. Copy the generated path function into your path_planner_client.py
"""

import svgpathtools
import numpy as np
from geometry_msgs.msg import Pose, PoseArray


class SVGToPumpkinPath:
    def __init__(self, svg_file, pumpkin_face_center, scale=0.001):
        """
        Convert SVG paths to pumpkin carving paths
        
        Args:
            svg_file: Path to SVG file
            pumpkin_face_center: Pose() of pumpkin center
            scale: Scale factor (default 0.001 = 1 SVG unit = 1mm)
        """
        self.svg_file = svg_file
        self.pumpkin_center = pumpkin_face_center
        self.scale = scale
        
        # Load SVG
        self.paths, self.attributes = svgpathtools.svg2paths(svg_file)
        
    def svg_point_to_pose(self, complex_point):
        """
        Convert SVG complex number point to ROS Pose
        SVG coordinate system: (0,0) at top-left, Y increases downward
        Pumpkin system: center at origin, Y increases upward
        """
        pose = Pose()
        
        # Extract x, y from complex number and scale
        x_offset = complex_point.real * self.scale
        y_offset = -complex_point.imag * self.scale  # Flip Y axis
        
        # Apply to pumpkin center position
        pose.position.x = self.pumpkin_center.position.x + x_offset
        pose.position.y = self.pumpkin_center.position.y + y_offset
        pose.position.z = self.pumpkin_center.position.z
        
        # Keep same orientation (z into pumpkin)
        pose.orientation = self.pumpkin_center.orientation
        
        return pose
    
    def path_to_pose_array(self, path, num_points_per_segment=10):
        """
        Convert a single SVG path to PoseArray
        
        Args:
            path: svgpathtools Path object
            num_points_per_segment: Points to sample per curve segment
        """
        pose_array = PoseArray()
        
        for segment in path:
            # Sample points along the segment
            for i in range(num_points_per_segment):
                t = i / num_points_per_segment
                point = segment.point(t)
                pose = self.svg_point_to_pose(point)
                pose_array.poses.append(pose)
        
        # Add final point to close the path
        final_point = path.point(1.0)
        pose_array.poses.append(self.svg_point_to_pose(final_point))
        
        return pose_array
    
    def convert_all_paths(self, num_points_per_segment=10):
        """
        Convert all paths in SVG to list of PoseArrays
        
        Returns:
            List of PoseArray objects, one per SVG path
        """
        pose_arrays = []
        
        for path in self.paths:
            if len(path) > 0:  # Skip empty paths
                pose_array = self.path_to_pose_array(path, num_points_per_segment)
                if len(pose_array.poses) > 0:
                    pose_arrays.append(pose_array)
        
        return pose_arrays
    
    def center_and_scale_to_fit(self, max_size=0.1):
        """
        Automatically center the SVG and scale it to fit within max_size
        
        Args:
            max_size: Maximum dimension in meters (default 10cm)
        """
        # Find bounding box of all paths
        all_points = []
        for path in self.paths:
            for segment in path:
                for t in np.linspace(0, 1, 10):
                    point = segment.point(t)
                    all_points.append((point.real, point.imag))
        
        if not all_points:
            return
        
        # Calculate bounds
        x_coords = [p[0] for p in all_points]
        y_coords = [p[1] for p in all_points]
        
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        # Calculate center offset (in SVG coordinates)
        x_center = (x_min + x_max) / 2
        y_center = (y_min + y_max) / 2
        
        # Calculate required scale
        svg_width = x_max - x_min
        svg_height = y_max - y_min
        svg_max_dim = max(svg_width, svg_height)
        
        if svg_max_dim > 0:
            self.scale = max_size / svg_max_dim
        
        # Center the paths
        for i, path in enumerate(self.paths):
            centered_path = path.translated(complex(-x_center, -y_center))
            self.paths[i] = centered_path


def generate_code_from_svg(svg_file, scale=0.001, auto_scale=True, max_size=0.1):
    """
    Generate Python code that can be copied into path_planner_client.py
    
    Args:
        svg_file: Path to SVG file
        scale: Manual scale factor (ignored if auto_scale=True)
        auto_scale: Automatically scale to fit max_size
        max_size: Maximum dimension in meters if auto_scaling
    """
    # Create dummy pumpkin center for processing
    from geometry_msgs.msg import Pose
    dummy_center = Pose()
    dummy_center.position.x = 0.5
    dummy_center.position.y = 0.0
    dummy_center.position.z = 0.3
    dummy_center.orientation.w = 1.0
    
    converter = SVGToPumpkinPath(svg_file, dummy_center, scale)
    
    if auto_scale:
        converter.center_and_scale_to_fit(max_size)
    
    pose_arrays = converter.convert_all_paths(num_points_per_segment=15)
    
    print(f"\n# Generated from {svg_file}")
    print(f"# Found {len(pose_arrays)} paths")
    print(f"# Scale factor: {converter.scale}")
    print("\ndef generate_svg_design(self):")
    print("    '''")
    print(f"    Generated from {svg_file}")
    print("    '''")
    print("    paths = []")
    print()
    
    for idx, pose_array in enumerate(pose_arrays):
        print(f"    # Path {idx + 1}")
        print(f"    path_{idx} = PoseArray()")
        
        for pose in pose_array.poses:
            x_offset = pose.position.x - dummy_center.position.x
            y_offset = pose.position.y - dummy_center.position.y
            
            print(f"    pose = Pose()")
            print(f"    pose.position.x = self.pumpkin_face_center.position.x + {x_offset:.6f}")
            print(f"    pose.position.y = self.pumpkin_face_center.position.y + {y_offset:.6f}")
            print(f"    pose.position.z = self.pumpkin_face_center.position.z")
            print(f"    pose.orientation = self.pumpkin_face_center.orientation")
            print(f"    path_{idx}.poses.append(pose)")
            print()
        
        print(f"    paths.append(path_{idx})")
        print()
    
    print("    return paths")
    print()
    print(f"# Total paths: {len(pose_arrays)}")
    print(f"# Total points: {sum(len(pa.poses) for pa in pose_arrays)}")


# Example usage
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python svg_to_path.py <svg_file> [max_size_in_meters]")
        print("\nExample: python svg_to_path.py jack_o_lantern.svg 0.15")
        print("         (creates paths that fit in 15cm square)")
        sys.exit(1)
    
    svg_file = sys.argv[1]
    max_size = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
    
    print(f"Converting {svg_file} to pumpkin carving paths...")
    print(f"Maximum size: {max_size}m ({max_size*100}cm)")
    
    generate_code_from_svg(svg_file, auto_scale=True, max_size=max_size)