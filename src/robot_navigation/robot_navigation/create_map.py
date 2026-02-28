import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import subprocess
import sys
from scipy.ndimage import gaussian_filter
from PIL import Image
import yaml

def draw_arena_square(map_size, square_length_pixels, border_thickness=1):
    """
    Creates a map with a centered square arena.
    Outside: Gray (128)
    Border: Black (0)
    Inside: White (255)
    """
    # Initialize map with 128 (Gray)
    map_array = np.full((map_size, map_size), 128, dtype=np.uint8)
    
    center = map_size // 2
    half_length = square_length_pixels // 2
    
    start = int(center - half_length)
    end = int(center + half_length)
    
    # Ensure bounds
    start = max(0, start)
    end = min(map_size, end)
    
    if start >= end:
        return map_array
        
    # Fill Inside with White
    map_array[start:end, start:end] = 255
    
    # Draw Borders (Black)
    if border_thickness > 0:
        # Top
        map_array[start:min(start+border_thickness, end), start:end] = 0
        # Bottom
        map_array[max(start, end-border_thickness):end, start:end] = 0
        # Left
        map_array[start:end, start:min(start+border_thickness, end)] = 0
        # Right
        map_array[start:end, max(start, end-border_thickness):end] = 0
    
    return map_array

def create_base_map(map_size=256):
    """
    Create a white map (free space).
    255 = Free space
    0 = Occupied
    """
    # Initialize map with 255 (White/Free)
    map_array = np.full((map_size, map_size), 255, dtype=np.uint8)
    return map_array

def add_corners(map_array, corner_data, pixels_per_meter):
    """
    Add black corners to the map.
    """
    map_size = map_array.shape[0]
    y_coords, x_coords = np.meshgrid(range(map_size), range(map_size))
    
    for corner_type, radius in corner_data:
        radius_pixels = int(radius * pixels_per_meter)
        
        if corner_type == 'TL':  # Top Left (0,0 in image coords usually top-left)
            # Note: In numpy (row, col) -> (y, x). 
            # If origin is bottom-left for ROS, we need to be careful.
            # Let's assume standard image coordinates for drawing, then flip for ROS if needed.
            # Standard Image: (0,0) is Top-Left.
            corner_x, corner_y = 0, 0
        elif corner_type == 'TR':  # Top Right
            corner_x, corner_y = map_size - 1, 0
        elif corner_type == 'BL':  # Bottom Left
            corner_x, corner_y = 0, map_size - 1
        elif corner_type == 'BR':  # Bottom Right
            corner_x, corner_y = map_size - 1, map_size - 1
            
        distances = np.sqrt((x_coords - corner_x)**2 + (y_coords - corner_y)**2)
        within_radius = distances <= radius_pixels
        
        # Set pixels within radius to 0 (Black/Occupied)
        map_array[within_radius] = 0
        
    return map_array

def open_in_gimp(image_path):
    """
    Opens the image in GIMP and waits for the process to close.
    """
    print(f"Opening {image_path} in GIMP...")
    try:
        # Wait for gimp to close
        subprocess.run(['gimp', image_path], check=True)
        print("GIMP closed. Reloading map...")
        return True
    except FileNotFoundError:
        print("Error: GIMP is not installed or not in PATH.")
        return False
    except Exception as e:
        print(f"Error running GIMP: {e}")
        return False

def save_ros_map(map_array, map_dir, resolution=0.005859,
                 map_size=None, real_size=None,
                 square_length=None, border_thickness=None):
    """
    Save map as .pgm and .yaml for ROS 2.  The additional optional
    parameters will be written back into the YAML metadata so that
    subsequent runs and the map node can pick them up.
    """
    # 1. Save PGM
    pgm_filename = "map.pgm"
    pgm_path = os.path.join(map_dir, pgm_filename)
    
    # Convert to image and save
    # ROS map_server expects: 0 (black) = occupied, 255 (white) = free, 205 = unknown
    # Our array is already in this format.
    img = Image.fromarray(map_array)
    # Rotate 90 degrees to fix orientation
    img_pgm = img.rotate(-90, expand=True)
    img_pgm.save(pgm_path)
    
    # 1.1 Save PNG for visualization
    png_filename = "map_view.png"
    png_path = os.path.join(map_dir, png_filename)
    img.save(png_path)

    # 2. Update YAML metadata file with our parameters
    yaml_path = os.path.join(map_dir, 'my_map.yaml')
    metadata = {}
    # try to load existing data to preserve unrelated fields
    if os.path.exists(yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                metadata = yaml.safe_load(f) or {}
        except Exception:
            metadata = {}
    # set/override values
    if map_size is not None:
        metadata['map_size'] = map_size
    if real_size is not None:
        metadata['real_size'] = real_size
    if square_length is not None:
        metadata['square_length'] = square_length
    if border_thickness is not None:
        metadata['border_thickness'] = border_thickness
    metadata['resolution'] = resolution
    # keep origin if already present otherwise calculate a default
    if 'origin' not in metadata:
        # default origin based on assuming center start
        msize = real_size if real_size else 1.5
        metadata['origin'] = [-msize/2.0, -msize/2.0, 0.0]
    # write file
    with open(yaml_path, 'w') as f:
        yaml.dump(metadata, f, default_flow_style=False)

    print(f"Saved ROS map files to {map_dir}")
    print(f"  - {pgm_filename}")
    print(f"  - map_view.png")
    print(f"  - {os.path.basename(yaml_path)} updated with parameters")

def get_map_dir():
    # Logic to find the correct directory (same as before)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    current_dir = os.getcwd()
    src_robot_navigation_dir = os.path.join(current_dir, 'src', 'robot_navigation')
    
    if os.path.exists(src_robot_navigation_dir):
        map_dir = os.path.join(src_robot_navigation_dir, 'map')
    else:
        workspace_root = script_dir
        while workspace_root != '/' and not os.path.exists(os.path.join(workspace_root, 'src')):
            workspace_root = os.path.dirname(workspace_root)
        
        if os.path.exists(os.path.join(workspace_root, 'src', 'robot_navigation')):
            map_dir = os.path.join(workspace_root, 'src', 'robot_navigation', 'map')
        else:
            map_dir = os.path.join(os.getcwd(), 'src', 'robot_navigation', 'map')
            
    os.makedirs(map_dir, exist_ok=True)
    return map_dir


def load_yaml_params(map_dir):
    """
    Load optional parameters from the YAML file stored alongside the map.
    If the file doesn't exist or a key is missing, defaults will be applied by
    the caller.
    """
    yaml_path = os.path.join(map_dir, 'my_map.yaml')
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        data = {}
    except Exception as e:
        print(f"Warning: could not read yaml parameters ({e})")
        data = {}
    return data

def parse_corners(corner_args):
    # ...existing code...
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
        description='Create a ROS 2 PGM map with corners and manual editing',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('corners', nargs='*', default=[],
                       help='Corner specifications in format CORNER:RADIUS (e.g., TL:0.5 BR:0.3)')
    parser.add_argument('--square-length', type=int,
                       help='Length of the square arena in pixels (overrides yaml)')
    parser.add_argument('--map-size', type=int,
                       help='Size of the map in pixels (width/height)')
    parser.add_argument('--real-size', type=float,
                       help='Physical size of the map in meters')
    parser.add_argument('--border-thickness', type=int,
                       help='Border thickness in pixels')
    
    args = parser.parse_args()
    corner_data = parse_corners(args.corners)
    
    # load yaml defaults and then override with command-line arguments
    map_dir = get_map_dir()
    yaml_params = load_yaml_params(map_dir)
    
    MAP_SIZE = args.map_size or yaml_params.get('map_size', 128)
    REAL_SIZE = args.real_size or yaml_params.get('real_size', 2.56)
    SQUARE_LENGTH = args.square_length or yaml_params.get('square_length', 54)
    BORDER_THICKNESS = args.border_thickness or yaml_params.get('border_thickness', 2)
    
    PIXELS_PER_METER = MAP_SIZE / REAL_SIZE
    RESOLUTION = yaml_params.get('resolution', REAL_SIZE / MAP_SIZE)
    
    print(f"Creating map with corners: {corner_data}")
    print(f"  map_size={MAP_SIZE}, real_size={REAL_SIZE}, square_length={SQUARE_LENGTH}, border={BORDER_THICKNESS}")
    
    # 1. Create Base Map (Gray outside, White inside square, Black border)
    square_length_pixels = int(SQUARE_LENGTH * PIXELS_PER_METER)
    map_array = draw_arena_square(MAP_SIZE, square_length_pixels , border_thickness=BORDER_THICKNESS)
    
    # 2. Add Corners
    map_array = add_corners(map_array, corner_data, PIXELS_PER_METER)
    
    # 3. (Border step removed)

    # Save temporary file for GIMP
    map_dir = get_map_dir()
    temp_img_path = os.path.join(map_dir, "temp_map_edit.png")
    temp_gimp_path = os.path.join(map_dir, "temp_map_edit.xcf")
    Image.fromarray(map_array).save(temp_img_path)
    
    #4. Ask user for obstacles
    response = input("Do you want to add obstacles manually in GIMP? (y/n): ").lower()
    
    if response == 'y':
        print("Opening GIMP. Please draw black lines/shapes for obstacles.")
        print("Save and Overwrite the file when done, then close GIMP.")
        if open_in_gimp(temp_img_path):
            # Reload the map
            img = Image.open(temp_img_path).convert('L') # Convert to grayscale
            map_array = np.array(img)

    # 5. Apply Gaussian Blur
    print("Applying Gaussian Blur...")
    # Sigma determines the amount of blur. 
    # 1.0 is subtle, 3.0 is strong.
    # This creates a cost gradient around obstacles.
    # However, for a standard static map, we usually want sharp edges.
    # If you want a costmap-like gradient baked in, keep the blur.
    # If you want a standard occupancy grid, we might threshold it back.
    
    # Let's apply a slight blur to smooth manual drawing edges, 
    # but keep it mostly binary for the static map server.
    blurred_map = gaussian_filter(map_array, sigma=1.0)
    
    # Thresholding to ensure clear occupied/free space for the static map
    # < 50 becomes 0 (Occupied), > 200 becomes 255 (Free), rest is grey
    # But user asked for blur, so we save the blurred version.
    # Note: map_server interprets [0, free_thresh) as occupied? No.
    # Standard: 0=free, 100=occupied. 
    # PGM: 0=black(occupied), 255=white(free).
    
    # 5. Save Final Map
    save_ros_map(blurred_map.astype(np.uint8), map_dir, RESOLUTION,
                 map_size=MAP_SIZE, real_size=REAL_SIZE,
                 square_length=SQUARE_LENGTH, border_thickness=BORDER_THICKNESS)
    
    # Cleanup
    if os.path.exists(temp_img_path):
        os.remove(temp_img_path)
    if os.path.exists(temp_gimp_path):
        os.remove(temp_gimp_path)
if __name__ == '__main__':
    main()
