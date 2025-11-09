
import numpy as np
import cv2

# from src.detection.math.pose import Pose
# from src.detection.calibration.calibration import Calibration
# from src.detection.detector import AprilTagDetector

from .aprilTagDetectorAlt import AprilTagDetector
from detection.math_funcs import *

import cv2.fisheye
import numpy as np
import json
import os

class Calibration:
    def __init__(self, options: dict):
        if 'camera_calibration_path' in options.keys():
            calibration_path = options['camera_calibration_path']
            print(f"Loading calibration from: {calibration_path}")
            ext = os.path.splitext(calibration_path)[1]
            if ext == '.json':
                self.load_json(calibration_path)
            elif ext == '.npz':
                self.load_npz(calibration_path)
            else:
                raise Exception(f"Unknown filetype: {ext} for file: {calibration_path}")
        else:
            self.load_default_calibration()
        self.print_calibration()

    def get(self):
        return self.calib

    def change_to_rectified_image(self, calibration_data):
        self.calib_original = self.calib.copy()
        self.calib = calibration_data

    def project(self, pt3d):
        if self.calib['useFisheyeModel'] and False:
            pt2d, _ = cv2.fisheye.projectPoints(objectPoints=pt3d,
                                                rvec=np.eye(3),
                                                tvec=np.zeros(3),
                                                  K=self.calib['camera_matrix'],
                                                  D=self.calib['distortion_coeffs'],)
        else:
            pt2d, _ = cv2.projectPoints(objectPoints=pt3d,
                                        rvec=np.eye(3),
                                        tvec=np.zeros(3),
                                        cameraMatrix=self.calib['camera_matrix'],
                                        distCoeffs=self.calib['distortion_coeffs'],)
        return pt2d

    def print_calibration(self):
        for k, v in self.calib.items():
            print(f"{k}: {v}")

        res_up = -self.calib['camera_matrix'][0][2]
        res_down = self.calib['resolution'][0] - self.calib['camera_matrix'][0][2]
        res_left = -self.calib['camera_matrix'][1][2]
        res_right = self.calib['resolution'][1] - self.calib['camera_matrix'][1][2]
        def compute_fov(f, res):
            return np.arctan(res / f) * 180 / np.pi
        fx = self.calib['camera_matrix'][0][0]
        fy = self.calib['camera_matrix'][1][1]
        fov_up = compute_fov(fx, res_up)
        fov_down = compute_fov(fx, res_down)
        fov_left = compute_fov(fy, res_left)
        fov_right = compute_fov(fy, res_right)

        print(f"Camera FoV: {fov_up:.2f}° up, {fov_down:.2f}° down, {fov_left:.2f}° left, {fov_right:.2f}° right")
        fov_vertical = fov_down - fov_up
        fov_horizontal = fov_right - fov_left
        print(f"Horizontal Fov: {fov_horizontal:.2f}°, Vertical Fov: {fov_vertical:.2f}°, Diagonal Fov: {np.sqrt(fov_horizontal**2 + fov_vertical**2):.2f}°")

    def load_npz(self, npz_file):
        self.calib = np.load(npz_file)

    def load_json(self, json_file):
        # Load JSON file
        with open(json_file, 'r') as f:
            json_data = json.load(f)

        self.calib = {key: np.array(value) for key, value in json_data.items()}

    def load_default_calibration(self):
        # Default calibration data
        resolution = np.array([480, 640])
        focal_length = 300.0
        self.calib = {
            'resolution': resolution,
            'camera_matrix': np.array([[focal_length, 0, resolution[1]/2],
                                       [0, focal_length, resolution[0]/2],
                                       [0, 0, 1.0]]),
            'distortion_coeffs': np.zeros((5,)),
            'useFisheyeModel': False,
            'mean_error': -1.0,
            'max_error': -1.0,
        }

    def convert_npz_to_json(self, npz_filepath):
        # Load the NPZ file
        data = np.load(npz_filepath, allow_pickle=True)

        # Convert numpy arrays to lists
        json_data = {key: data[key].squeeze().tolist() for key in data}

        # Generate JSON file path
        json_filepath = os.path.splitext(npz_filepath)[0] + ".json"

        # Save as JSON
        with open(json_filepath, 'w') as f:
            json.dump(json_data, f, indent=4)
        print(f"JSON saved to {json_filepath}")

arrow_forward_x = 0.2 * np.array([[0, 0, 0],
                [1, 0, 0],
                [0.75, 0.25, 0],
                [1, 0, 0],
                [0.75, -0.25, 0],
                [1, 0, 0]])


def draw_outline(outline, image, text=None, color=(0, 255, 0)):
    for i in range(len(outline)):
        pt1 = tuple(outline[i].astype(int))
        pt2 = tuple(outline[(i + 1) % len(outline)].astype(int))
        cv2.line(image, pt1, pt2, color, 2)
        cv2.putText(image, f"{i}", pt1,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    if text:
        cv2.putText(image, text, outline[0].astype(int),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

class RobotDetector():
    def __init__(self, options: dict, calibration):
        self.robot_tags = options['robot_tags']
        self.robot_outline = 0.13 * np.array([[0,-4,0],
                                              [2,0,0],
                                              [2, 1, 0],
                                              [-2, 1, 0],
                                              [-2, 0, 0]])
        self.detection_last = None
        self.calibration = calibration
        self.april_tag_detector = AprilTagDetector(self.calibration)

    def detect(self, image_gray, worldFromCamera: Pose, debug_image=None):
        # Detect Robot
        # print(f'Detecting robot...')

        april_tag_detections = self.april_tag_detector.detect(image_gray, self.robot_tags['sizes'])['aprilTags']
        # for tag_detection in april_tag_detections:
        #     print(f'robotDetector: detected tag {tag_detection.tag_id} of family {tag_detection.tag_family.decode()} with pose error {tag_detection.pose_err:.3f} m')

        for tag_detection in april_tag_detections:
            if (tag_detection.tag_family.decode() == self.robot_tags['family'] and
                    (tag_detection.tag_id == self.robot_tags['top_id'] or
                     tag_detection.tag_id == self.robot_tags['bottom_id'])):
                if tag_detection.pose_err > 1e-3:
                    print(f"Detection error is large: {tag_detection.pose_err:.3f} m")

                detection = {}
                detection['cameraFromRobot'] = Pose(tag_detection.pose_R, tag_detection.pose_t)
                detection['robotInCamera2D'] = self.calibration.project(tag_detection.pose_t).squeeze()
                detection['worldFromRobot'] = worldFromCamera * detection['cameraFromRobot']

                # Compute the robot forward arrow in 2D
                robotForwardArrowInCamera2D = []
                for arrow_point in arrow_forward_x:
                    arrowPoseInRobot = Pose(np.eye(3), arrow_point)
                    robotForwardArrowInCamera2D.append(self.calibration.project(
                        (detection['cameraFromRobot'] * arrowPoseInRobot).t).squeeze())

                # Compute the robot outline in 3D and 2D
                robotOutlineInCamera3D = []
                robotOutlineInCamera2D = []
                robotOutlineInWorld3D = []

                for outline_point in self.robot_outline:
                    outlineInRobot = Pose(np.eye(3), outline_point)
                    robotOutlineInCamera3D.append(detection['cameraFromRobot'] * outlineInRobot)
                    robotOutlineInCamera2D.append(self.calibration.project(robotOutlineInCamera3D[-1].t).squeeze())
                    robotOutlineInWorld3D.append(detection['worldFromRobot'] * outlineInRobot)
                detection['robotOutlineInCamera2D'] = robotOutlineInCamera2D

                # Compute the tag outline in 3D and 2D
                tag_outline = self.april_tag_detector.tag_outline()
                tagOutlineInCamera3D = []
                tagOutlineInCamera2D = []

                for outline_point in tag_outline:
                    outlineInRobot = Pose(np.eye(3), outline_point)
                    tagOutlineInCamera3D.append(detection['cameraFromRobot'] * outlineInRobot)
                    tagOutlineInCamera2D.append(self.calibration.project(tagOutlineInCamera3D[-1].t).squeeze())

                if debug_image is not None:
                    # self.april_tag_detector.draw_in_image(detection, debug_image)
                    # draw_outline(tagOutlineInCamera2D, debug_image, 'tagProjected', (0, 255, 0))
                    cv2.circle(debug_image, detection['robotInCamera2D'].astype(int), 5, (0, 255, 0))
                    draw_outline(robotForwardArrowInCamera2D, debug_image, 'forward', (255, 0, 0))
                    draw_outline(robotOutlineInCamera2D, debug_image, 'robot', (0, 255, 0))

                    cv2.imshow("RobotDetector Debug Image", debug_image)

                self.detection_last = detection
                return detection

        print(f'RobotDetector: Could not find robot')
        if debug_image is not None:
            if self.detection_last is not None:
                draw_outline(self.detection_last['robotOutlineInCamera2D'] , debug_image, 'robot', (0, 0, 255))
            cv2.imshow("RobotDetector Debug Image", debug_image)
        return self.detection_last