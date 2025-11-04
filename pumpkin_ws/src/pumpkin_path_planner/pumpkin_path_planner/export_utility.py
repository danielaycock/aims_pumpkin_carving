#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2025, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# AUTHOR: A.C. Buynak


# General Imports
from datetime import datetime
from pathlib import Path
import re
import yaml

# ROS & Project Specific Messages
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, Pose
from builtin_interfaces.msg import Time, Duration


## Export Utility Class
class ExportUtility:
    """
    Utility for exporting path planner results to various formats and tools.
    """

    log_name = __name__
    data_directory_filepath = ".aims"
    logger = print

    def __init__(self, log_name: str, data_directory: str = ".aims", logger=print):
        """
        :param log_name: Name of subfolder
        :param data_directory: Name of main directory to store all logs
        :param logger: Logging function. If using ROS Logger, you MUST pass get_logger().info or .warn without parentheses.
        """
        self.log_name = log_name + "_" + datetime.now().strftime("%Y%m%d-%H%M%S")
        self.data_directory_filepath = (
            Path(Path.home()).joinpath(data_directory).joinpath(self.log_name)
        )
        self.data_directory_filepath.mkdir(parents=True, exist_ok=True)
        self.logger = logger

    def _pose_to_dict(self, pose: Pose):
        """
        Function to convert a Pose object to a dictionary for YAML serialization
        """
        return {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
        }

    def _pose_array_to_dict(self, pose_array: PoseArray):
        """
        Function to convert a PoseArray object to a dictionary for YAML serialization
        """
        return {
            "header": {
                "stamp": {
                    "sec": pose_array.header.stamp.sec,
                    "nanosec": pose_array.header.stamp.nanosec,
                },
                "frame_id": pose_array.header.frame_id,
            },
            "poses": [self._pose_to_dict(p) for p in pose_array.poses],
        }


    def _joint_trajectory_point_to_dict(self, point: JointTrajectoryPoint):
        """
        Helper method to convert a JointTrajectoryPoint object to a dictionary for YAML serialization

        :param point (JointTrajectoryPoint): Single trajectory point ROS 2 message
        """
        return {
            "positions": list(point.positions),
            "velocities": list(point.velocities),
            "accelerations": list(point.accelerations),
            "effort": list(point.effort),
            "time_from_start": {
                "sec": point.time_from_start.sec,
                "nanosec": point.time_from_start.nanosec,
            },
        }

    def _joint_trajectory_to_dict(self, msg: JointTrajectory):
        """
        Function to convert a PoseArray object to a dictionary for YAML serialization

        :param msg (JointTrajectory): Joint Trajectory ROS 2 message
        """

        converted_dict = {
            "header": {
                "stamp": {
                    "sec": msg.header.stamp.sec,
                    "nanosec": msg.header.stamp.nanosec,
                },
                "frame_id": msg.header.frame_id,
            },
            "joint_names": list(msg.joint_names),
            "points": [],
        }

        for point in msg.points:
            # Convert each JointTrajectoryPoint to a dictionary
            point_dict = self._joint_trajectory_point_to_dict(point)
            converted_dict["points"].append(point_dict)
        return converted_dict

    def _generate_filepath(self, filename: str, file_extension: str) -> str:
        """
        Generate a valid path for the directory.

        :param file_name: Name of file to writeout.
        :param file_extension: File format string to append to end of generated file.
        :return Generated filepath
        """
        filepath = Path(
            self.data_directory_filepath, filename + "_00." + file_extension
        )

        # Ensure file is unique, if not append a number
        if Path.is_file(filepath):

            prefix = filename + "_"

            highest_number = -1
            pattern = re.compile(
                f"^{re.escape(prefix)}(\d+).*{re.escape(file_extension)}", re.IGNORECASE
            )

            for file_path in Path(self.data_directory_filepath).rglob(
                f"{prefix}*{file_extension}"
            ):
                match = pattern.match(file_path.name)
                if match:
                    number_str = match.group(1)
                    try:
                        current_number = int(number_str)
                        if current_number > highest_number:
                            highest_number = current_number
                    except ValueError:
                        # Handle cases where the extracted string is not a valid integer
                        self.logger(
                            f"Warning: Could not convert '{number_str}' to an integer from file '{filename}'."
                        )

            filepath = Path.joinpath(
                self.data_directory_filepath,
                filename + "_" + highest_number + 1 + "." + file_extension,
            )

        return filepath

    # Function to write the JointTrajectory data structure to a YAML file
    def write_joint_trajectory_to_yaml(self, msg: JointTrajectory):
        data = self._joint_trajectory_to_dict(msg)
        filepath = self._generate_filepath("joint_trajectory", "yaml")

        with Path.open(filepath, "x") as f:
            yaml.dump(data, f, default_flow_style=False)

        self.logger(f"Wrote out JointTrajectory to file {filepath}")


    # Function to write the PoseArray data structure to a YAML file
    def write_pose_array_to_yaml(self, tool_path: PoseArray):
        data = self._pose_array_to_dict(tool_path)
        filepath = self._generate_filepath("pose_array", "yaml")

        with Path.open(filepath, "x") as f:
            yaml.dump(data, f, default_flow_style=False)

        self.logger(f"Wrote out PoseArray to file {filepath}")


    # Function to write the List of PoseArray data structure to a YAML file
    def write_pose_array_list_to_yaml(self, tool_path: list):
        data = {
            "path": [self._pose_array_to_dict(pa) for pa in tool_path]
        }
        filepath = self._generate_filepath("tool_path", "yaml")

        with Path.open(filepath, "x") as f:
            yaml.dump(data, f, default_flow_style=False)

        self.logger(f"Wrote out ToolPath (list of pose arrays) to file {filepath}")



# Example usage
if __name__ == "__main__":

    msg = JointTrajectory()
    msg.header.stamp = Time(sec=0, nanosec=0)
    msg.header.frame_id = "world"
    msg.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    # First trajectory point
    point1 = JointTrajectoryPoint()
    point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point1.time_from_start = Duration(sec=0, nanosec=0)
    msg.points.append(point1)
    
    # Second trajectory point
    point2 = JointTrajectoryPoint()
    point2.positions = [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    point2.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point2.time_from_start = Duration(sec=10, nanosec=0)
    msg.points.append(point2)

    # Third trajectory point
    point3 = JointTrajectoryPoint()
    point3.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point3.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point3.time_from_start = Duration(sec=20, nanosec=0)
    msg.points.append(point3)

    # Write the data to a YAML file
    ExportUtility("log").write_joint_trajectory_to_yaml(msg)

# EOF