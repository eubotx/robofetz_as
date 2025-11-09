import cv2
import numpy as np
import json
import detection.frame_source


# Global variables
points = []
arena_width =400  # cm (width of the arena)
arena_height = 400  # cm (height of the arena)

options = {
    # Calibration
    'camera_calibration_path': 'src/detection/data/USBGS720P02-L170_calibration.json',
    # ROS
    'publish_ros': False,
    # Video
    # 'frame_path': '/home/tiago/Documents/CamTestFootage/vids_position_top_down/output_bot_april_corners.mp4',
    'frame_path': 'src/detection/recordings/output_april_corner_movement.mp4',
    # Webcam
    'webcam_id': 4,
    'webcam_save_stream_path': 'src/detection/recordings/testSeqxxx.mp4',
    # Tags
    'arena_tag': {'id': 2, 'family': 'tagStandard41h12'},
    'robot_tag': {'id': 12, 'family': 'tagStandard41h12'},
}

 # frame_source = frame_source.GenericSource(frame_source.SourceType.Video, options=options)
frame_source = detection.frame_source.GenericSource(detection.frame_source.SourceType.Webcam, options=options)

# Function to handle mouse click events
def click_event(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        # Store the points where the user clicks
        points.append((x, y))
        # Draw a circle at the clicked position
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Arena", frame)

        # If 4 points are selected, perform perspective transformation
        if len(points) == 4:
            perform_perspective_transform()

# Function to perform perspective transformation
def perform_perspective_transform():
    global points, frame
    # Define the four points of the arena in the current view
    src_points = np.float32(points)

    # Define the four points in the "top-down" view (ceiling-mounted view)
    dst_points = np.float32([
        [0, 0],  # Top-left corner
        [arena_width, 0],  # Top-right corner
        [arena_width, arena_height],  # Bottom-right corner
        [0, arena_height]  # Bottom-left corner
    ])

    # Get the perspective transform matrix
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Save the data for later use
    save_transformation_data(src_points, matrix)

    # Perform the perspective transformation
    transformed_image = cv2.warpPerspective(frame, matrix, (arena_width, arena_height))

    # Display the transformed image
    cv2.imshow("Transformed Arena", transformed_image)

# Function to save transformation data to a file
def save_transformation_data(src_points, matrix):
    # Save data as a dictionary
    transformation_data = {
        "src_points": src_points.tolist(),  # Convert NumPy array to a list for JSON serialization
        "matrix": matrix.tolist()  # Convert NumPy array to a list
    }

    # Save the data to a JSON file
    with open("src/detection/data/transformation_data.json", "w") as f:
        json.dump(transformation_data, f)
    print("Transformation data saved to 'transformation_data.json'.")

# Set up mouse callback
cv2.namedWindow("Arena")
cv2.setMouseCallback("Arena", click_event)

while True:
    # Capture frame from the webcam
    print('Getting frame...')
    frame = frame_source.get_frame()
    if frame is None:
        break

    # Show the original frame
    cv2.imshow("Arena", frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all OpenCV windows

cv2.destroyAllWindows()
