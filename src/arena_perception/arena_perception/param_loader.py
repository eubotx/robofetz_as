#!/usr/bin/env python3
#filename: src/arena_perception/arena_perception/param_loader.py

import yaml
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

class ParamLoader:
    """Utility class to load parameters from YAML file"""
    
    def __init__(self, node: Node, config_path: str):
        """
        Initialize parameter loader
        
        Args:
            node: ROS2 node instance
            config_path: Full path to the YAML config file
        """
        self.node = node
        self.params = {}
        
        # Check if file exists
        if not os.path.exists(config_path):
            node.get_logger().error(f"Config file not found: {config_path}")
            return
            
        # Load YAML file
        with open(config_path, 'r') as f:
            self.params = yaml.safe_load(f)
            
        node.get_logger().info(f"Loaded parameters from {config_path}")
    
    def get_param(self, *keys, default=None):
        """
        Get parameter value using nested keys
        
        Example: get_param('kalman_filter', 'process_noise', 'position')
        """
        value = self.params
        for key in keys:
            if isinstance(value, dict):
                value = value.get(key)
                if value is None:
                    return default
            else:
                return default
        return value
    
    def declare_parameters_from_dict(self, prefix: str, param_dict: dict):
        """
        Recursively declare parameters from a dictionary
        """
        for key, value in param_dict.items():
            full_key = f"{prefix}.{key}" if prefix else key
            
            if isinstance(value, dict):
                self.declare_parameters_from_dict(full_key, value)
            else:
                self.node.declare_parameter(full_key, value)
    
    def get_parameters_with_prefix(self, prefix: str) -> dict:
        """
        Get all parameters with a given prefix as a nested dictionary
        """
        result = {}
        prefix_len = len(prefix) + 1 if prefix else 0
        
        for param_name in self.node.list_parameters([]).names:
            if param_name.startswith(prefix):
                # Split into nested structure
                parts = param_name[prefix_len:].split('.')
                value = self.node.get_parameter(param_name).value
                
                # Build nested dict
                current = result
                for part in parts[:-1]:
                    if part not in current:
                        current[part] = {}
                    current = current[part]
                current[parts[-1]] = value
                
        return result