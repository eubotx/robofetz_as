#!/usr/bin/env python3

import numpy as np
import yaml
import os

# Load your calibration files from current directory
K = np.load('K.npy')  # Camera matrix
D = np.load('D.npy')  # Distortion coefficients  
dims = np.load('Dims.npy')  # Image dimensions [width, height]

print(f"Loaded calibration:")
print(f"Image dimensions: {dims}")
print(f"Camera matrix K:\n{K}")
print(f"Distortion coefficients D: {D}")

# Convert to match your desired format
calibration = {
    'image_width': int(dims[0]),
    'image_height': int(dims[1]),
    'camera_name': 'arena_camera',
    'camera_matrix': {
        'rows': 3,
        'cols': 3,
        'data': [float(x) for x in K.flatten()]
    },
    'distortion_model': 'plumb_bob',
    'distortion_coefficients': {
        'rows': 1,
        'cols': len(D.flatten()),
        'data': [float(x) for x in D.flatten()]
    },
    'rectification_matrix': {
        'rows': 3,
        'cols': 3,
        'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    },
    'projection_matrix': {
        'rows': 3,
        'cols': 4,
        'data': [
            float(K[0,0]), 0.0, float(K[0,2]), 0.0,
            0.0, float(K[1,1]), float(K[1,2]), 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
    }
}

# Save as YAML in config directory
config_path = 'convert_output.yaml'
with open(config_path, 'w') as f:
    yaml.dump(calibration, f, default_flow_style=False, sort_keys=False)

print(f"Calibration converted to {config_path}")