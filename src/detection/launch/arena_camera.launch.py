#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('detection'),
        'config',
        'arena_camera_params.yaml'
    )

    camera_info = os.path.join(
        get_package_share_directory('detection'),
        'config',
        'arena_camera.info.yaml'
    )
    
    # Default parameters (good starting point)
    default_params = {
        # Hardware
        'video_device': '/dev/video0',                    # string: device path
        'camera_name': 'arena_camera',                    # string: camera name
        'camera_info_url': camera_info,                            # string: calibration file
        'frame_id': 'arena_camera_frame',                 # string: TF frame
        
        # Image format
        'image_width': 640,                               # int: resolution width
        'image_height': 480,                              # int: resolution height  
        'framerate': 100.0,                               # float: target FPS
        'pixel_format': 'yuyv',                           # string: 'yuyv', 'mjpeg', 'grey' -> 'yuyv' often also used in greyscale cams
        
        # Exposure (critical for FPS)
        'autoexposure': False,                            # bool: true, false
        'exposure_absolute': 156,                         # int: 1-8188 (default: 156) - increase if too dark, decrease if fps too slow
        'auto_gain': False,                               # bool: true=auto, false=manual
        'gain': 32,                                       # int: 16-255 (default: 32) - decrease if too noisy
        
        # Image quality
        'brightness': 4,                                  # int: -64 to 64 (default: 4)
        'contrast': 20,                                   # int: 0-95 (default: 20)
        'saturation': 0,                                  # int: 0-100 (default: 0) - keep 0 for greyscale
        'sharpness': 0,                                   # int: 0-7 (default: 0)
        'gamma': 100,                                     # int: 100-300 (default: 100)
        'hue': 0,                                         # int: -2000 to 2000 (default: 0)
        
        # White balance (disable for greyscale)
        'auto_white_balance': False,                      # bool: default=1 (true) - set false for greyscale
        'white_balance': 4600,                            # int: 2800-6500K (default: 4600)
        
        # Hardware specific
        'power_line_frequency': 1,                        # int: 0=disabled, 1=50Hz, 2=60Hz (default: 1)
        'backlight_compensation': 112,                    # int: 28-201 (default: 112)
        
        # Compression
        'use_image_transport': True,                      # bool: enable compressed topics to save bandwidth
        'jpeg_quality': 80,                               # int: 0-100 JPEG quality
        'publish_rate': 100.0,                            # float: publishing rate of raw topic
    }
    
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            #name='arena_camera_node',
            namespace='arena_camera',
            parameters=[default_params, config],  # YAML overrides defaults
        )
    ])