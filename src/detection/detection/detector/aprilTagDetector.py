import numpy as np
import pyapriltags as apriltag

class AprilTagDetector:
    def __init__(self, calibration_data):
        # calibration_data is now always a dict from camera_info
        camera_matrix = calibration_data['camera_matrix']
            
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        self.camera_intrinsics = np.array([fx, fy, cx, cy])

        # Initialize the AprilTag detector
        families = 'tagStandard41h12'
        self.detector = apriltag.Detector(families=families)

    def detect(self, image_gray, tag_size: float):
        detections = self.detector.detect(image_gray,
                                     estimate_tag_pose=True,
                                     camera_params=self.camera_intrinsics,
                                     tag_size=tag_size)
        return {"aprilTags": detections}