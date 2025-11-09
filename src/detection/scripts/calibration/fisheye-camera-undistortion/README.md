# fisheye-camera-undistortion
Fisheye camera distortion correction based on opencv's chessboard calibration algorithm
### Environment

- python==3.6.5
- opencv-python==4.2.0


### Getting Started

1. Run 'capture_calibration_images.py' to take several standard chessboard images with the fisheye lens to be corrected and place them into the 'calibration_images' folder.

2. Run 'camera_calibrate.py' to calculate the internal parameter matrix K and the distortion coefficient vector D.

3. Run 'image_correction.py' to correct a single image 'distorted.jpg' captured by the camera. Return 'undistorted.jpg'

4. Run 'video_correction.py' to correct the camera in real time. Set correct camera source!
