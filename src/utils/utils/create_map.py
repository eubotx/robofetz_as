# This script creates a simple map for the simulation environment

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

def create_map(corner_data):
    """
    Create a 256x256 pixel map representing a 1.5x1.5m area.
    
    Args:
        corner_data (list): List of tuples (corner_type, radius)
    
    Returns:
        np.ndarray: 256x256 normalized map array
    """
    # Map dimensions
    map_size = 256  # pixels
    real_size = 1.5  # meters
    pixels_per_meter = map_size / real_size
    
    # Initialize map with 0.5 values
    map_array = np.full((map_size, map_size), 0.5, dtype=np.float32)
    
    # Create coordinate grids
    y_coords, x_coords = np.meshgrid(range(map_size), range(map_size))
    
    # Process each corner
    for corner_type, radius in corner_data:
        # Convert radius to pixels
        radius_pixels = int(radius * pixels_per_meter)
        
        # Define corner coordinates based on corner type
        if corner_type == 'TL':  # Top Left
            corner_x, corner_y = 0, 0
        elif corner_type == 'TR':  # Top Right
            corner_x, corner_y = map_size - 1, 0
        elif corner_type == 'BL':  # Bottom Left
            corner_x, corner_y = 0, map_size - 1
        elif corner_type == 'BR':  # Bottom Right
            corner_x, corner_y = map_size - 1, map_size - 1
        else:
            raise ValueError("Corner type must be 'TL', 'TR', 'BL', or 'BR'")
        
        # Calculate distance from each pixel to the corner
        distances = np.sqrt((x_coords - corner_x)**2 + (y_coords - corner_y)**2)
        
        # Create mask for pixels within the radius
        within_radius = distances <= radius_pixels
        
        # For pixels within radius, decrease values based on distance
        # Values decrease from current value to 0.0 as we get closer to the corner
        normalized_distances = distances / radius_pixels
        normalized_distances = np.clip(normalized_distances, 0, 1)
        
        # Apply the effect: closer to corner = lower value
        # Use minimum to ensure multiple corners don't increase values
        corner_effect = map_array[within_radius] * normalized_distances[within_radius]
        map_array[within_radius] = np.minimum(map_array[within_radius], corner_effect)
    
    return map_array

def save_map(map_array, corner_data):
    """
    Save the map array to the utils/map directory.
    
    Args:
        map_array (np.ndarray): The map array to save
        corner_data (list): List of tuples (corner_type, radius)
    """
    # Find the source directory by looking for the workspace structure
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # When running with ros2 run, we're in install/utils/lib/python3.x/site-packages/
    # We need to find the src/utils/ directory instead
    current_dir = os.getcwd()  # This should be the workspace root
    
    # Look for src/utils directory from current working directory
    src_utils_dir = os.path.join(current_dir, 'src', 'utils')
    
    if os.path.exists(src_utils_dir):
        map_dir = os.path.join(src_utils_dir, 'map')
    else:
        # Fallback: try to navigate from script location
        # From install/utils/lib/python3.x/site-packages/ go back to workspace root
        workspace_root = script_dir
        while workspace_root != '/' and not os.path.exists(os.path.join(workspace_root, 'src')):
            workspace_root = os.path.dirname(workspace_root)
        
        if os.path.exists(os.path.join(workspace_root, 'src', 'utils')):
            map_dir = os.path.join(workspace_root, 'src', 'utils', 'map')
        else:
            # Last resort: create in current directory
            map_dir = os.path.join(os.getcwd(), 'src', 'utils', 'map')
    
    os.makedirs(map_dir, exist_ok=True)
    
    # Create filename from corner data
    corner_str = "_".join([f"{corner}{radius:.1f}" for corner, radius in corner_data])
    
    # Save as numpy array
    filename = f"map_robofetz.npy"
    filepath = os.path.join(map_dir, filename)
    np.save(filepath, map_array)
    
    # Also save as image for visualization
    img_filename = f"map_robofetz.png"
    img_filepath = os.path.join(map_dir, img_filename)
    
    plt.figure(figsize=(8, 8))
    plt.imshow(map_array, cmap='gray', origin='lower', vmin=0, vmax=1)
    plt.colorbar(label='Normalized Value')
    
    # Create title with all corners
    title_parts = [f"{corner}({radius:.1f}m)" for corner, radius in corner_data]
    title = f'Robofetz Map - Corners: {", ".join(title_parts)}'
    plt.title(title)
    plt.xlabel('X (pixels)')
    plt.ylabel('Y (pixels)')
    plt.savefig(img_filepath, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"Map saved as: {filepath}")
    print(f"Visualization saved as: {img_filepath}")

def parse_corners(corner_args):
    """
    Parse corner arguments into list of (corner_type, radius) tuples.
    
    Args:
        corner_args (list): List of strings in format "CORNER:RADIUS"
    
    Returns:
        list: List of tuples (corner_type, radius)
    """
    corner_data = []
    
    for arg in corner_args:
        if ':' not in arg:
            raise ValueError(f"Invalid format '{arg}'. Use CORNER:RADIUS format (e.g., TL:0.5)")
        
        corner_type, radius_str = arg.split(':')
        
        if corner_type not in ['TL', 'TR', 'BL', 'BR']:
            raise ValueError(f"Invalid corner type '{corner_type}'. Must be TL, TR, BL, or BR")
        
        try:
            radius = float(radius_str)
        except ValueError:
            raise ValueError(f"Invalid radius '{radius_str}'. Must be a number")
        
        if radius <= 0 or radius > 1.5:
            raise ValueError(f"Radius {radius} must be between 0 and 1.5 meters")
        
        corner_data.append((corner_type, radius))
    
    return corner_data

def main():
    parser = argparse.ArgumentParser(
        description='Create a map with corner effects',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s TL:0.5                    # Single corner: Top-left with 0.5m radius
  %(prog)s TL:0.3 BR:0.4             # Two corners: Top-left (0.3m) and Bottom-right (0.4m)
  %(prog)s TL:0.2 TR:0.2 BL:0.2 BR:0.2  # All four corners with 0.2m radius each
        """
    )
    
    parser.add_argument('corners', nargs='+', 
                       help='Corner specifications in format CORNER:RADIUS (e.g., TL:0.5 BR:0.3)')
    
    args = parser.parse_args()
    
    # Parse corner arguments
    corner_data = parse_corners(args.corners)
    
    print(f"Creating map with corners: {corner_data}")
    
    # Create the map
    map_array = create_map(corner_data)
    
    # Save the map
    save_map(map_array, corner_data)
    
    # Display statistics
    print(f"\nMap Statistics:")
    print(f"Size: {map_array.shape}")
    print(f"Min value: {map_array.min():.3f}")
    print(f"Max value: {map_array.max():.3f}")
    print(f"Mean value: {map_array.mean():.3f}")

if __name__ == '__main__':
    main()


# ros2 run utils create_map TL:0.3 BR:0.4
