import cv2
import numpy as np
import pyapriltags as apriltag  # Documentation: https://github.com/WillB97/pyapriltags

class AprilTagDetector:
    def __init__(self, calibration):
        camera_matrix = calibration.get()['camera_matrix']
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        self.camera_intrinsics=np.array([fx, fy, cx, cy])

        # Initialize the AprilTag detector
        # families = 'tag16h5 tag25h9 tag36h11 tagCircle21h7 tagCircle49h12 tagCustom48h12 tagStandard41h12 tagStandard52h13'
        families = 'tagStandard41h12'
        self.detector = apriltag.Detector(families=families)

    def tag_outline(self):
        # Define the real-world size of the AprilTag (in meters or any unit)
        APRILTAG_SIZE = 0.150
        DETECTION_SCALE_RATIO = 5 / 9  # Detection is only on the inside area while measurement was taken on outside of tag
        DETECTION_SCALE_RATIO = 1

        # Define object points for a single AprilTag
        april_tag_outline = np.array([
            [0, APRILTAG_SIZE, 0],
            [APRILTAG_SIZE, APRILTAG_SIZE, 0],
            [APRILTAG_SIZE, 0, 0],
            [0, 0, 0]
        ], dtype=np.float32)
        april_tag_outline = DETECTION_SCALE_RATIO * (
                    april_tag_outline - np.array([APRILTAG_SIZE / 2, APRILTAG_SIZE / 2, 0]))

        return april_tag_outline

    def draw_in_image(self, detection, debug_image=None):
        if debug_image is not None:
            for i in range(4):
                pt1 = tuple(detection.corners[i].astype(int))
                pt2 = tuple(detection.corners[(i + 1) % 4].astype(int))
                cv2.line(debug_image, pt1, pt2, (0, 255, 0), 2)
                cv2.putText(debug_image, f"{i}", pt1,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # Draw the center of the tag
            center = tuple(detection.center.astype(int))
            cv2.circle(debug_image, center, 5, (0, 0, 255), -1)

            # Display tag ID
            cv2.putText(debug_image, f"ID: {detection.tag_id}", (center[0] - 10, center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return debug_image

    def detect(self, image_gray, tag_size: float, debug_image=None):
        # Detect AprilTags
        detections = self.detector.detect(image_gray,
                                     estimate_tag_pose=True,
                                     camera_params=self.camera_intrinsics,
                                     tag_size=tag_size)
        if debug_image is None:
            return {"aprilTags": detections}

        print(f"Detected AprilTags: {len(detections)}.")
        for detection in detections:
            self.draw_in_image(detection, debug_image)
        cv2.imshow("AprilTagDetector Debug Image", debug_image)

        return {"aprilTags": detections}