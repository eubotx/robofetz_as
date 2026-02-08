#!/usr/bin/env python3
import cv2
import sys

# Script to resize the april tag to than use in the models or elsewhere
# example usage: python3 resize_apriltag.py apriltag-imgs/tagStandard41h12/tag41_12_00012.png top_apriltag.png

# Texture size must be power of two
TARGET_SIZE = 2048

def resize_tag(input_path, output_path, size):
    """
    Load a tag image and resize it to specified size.
    
    Args:
        input_path: Path to input PNG
        output_path: Path to save resized PNG
        size: Target size (width and height in pixels)
    """
    # Load the image in grayscale
    img = cv2.imread(input_path, 0)  # 0 = grayscale mode
    
    if img is None:
        print(f"Error: Could not load image {input_path}")
        return False
    
    # Resize to target size
    resized = cv2.resize(img, (size, size), interpolation=cv2.INTER_NEAREST)
    
    # Save resized image
    cv2.imwrite(output_path, resized)
    
    print(f"Resized: {input_path} -> {output_path}")
    print(f"  Original: {img.shape[1]}x{img.shape[0]}")
    print(f"  Resized:  {size}x{size}")
    return True

def main():
    # Example usage
    if len(sys.argv) == 3:
        # Use command line arguments
        input_png = sys.argv[1]
        output_png = sys.argv[2]
    else:
        print("Please provide input_path, output_path")
    
    resize_tag(input_png, output_png, TARGET_SIZE)

if __name__ == '__main__':
    main()