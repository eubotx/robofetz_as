#ros2 run your_package_name sam2_yolo_generator.py --ros-args -p video_folder:=/path/to/videos -p target_fps:=7 -p sam2_model:=sam2.1_b.pt

#!/usr/bin/env python3

import os
import cv2
import numpy as np
import torch
import random
import shutil
from tqdm import tqdm
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Import Ultralytics components
try:
    from ultralytics.models.sam import SAM2VideoPredictor
    from ultralytics.utils.ops import masks2segments
except ImportError:
    print("Ultralytics not installed. Please install it with: pip install ultralytics>=8.2")
    raise

class SAM2YOLOGenerator:
    # (Original class unchanged, but we'll add a method to check if dataset already exists)
    def __init__(self, video_path, output_dir, sam2_model="sam2_b.pt", target_fps=10):
        self.video_path = video_path
        self.output_dir = output_dir
        self.frames_dir = os.path.join(output_dir, "frames")
        self.dataset_dir = os.path.join(output_dir, "dataset")
        self.sam2_model = sam2_model
        self.target_fps = target_fps

        for directory in [self.output_dir, self.frames_dir, self.dataset_dir]:
            os.makedirs(directory, exist_ok=True)

        for split in ['train', 'val']:
            for subdir in ['images', 'labels']:
                os.makedirs(os.path.join(self.dataset_dir, split, subdir), exist_ok=True)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")

        self.class_map = {}

        cap = cv2.VideoCapture(self.video_path)
        self.original_fps = cap.get(cv2.CAP_PROP_FPS)
        self.frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()

        self.frame_interval = max(1, int(self.original_fps / self.target_fps))
        self.estimated_frames = self.frame_count // self.frame_interval

        print(f"Video properties: {self.frame_count} frames, {self.original_fps:.1f} FPS, {self.width}x{self.height}")
        print(f"Downsampling to {self.target_fps} FPS (extracting 1 frame every {self.frame_interval} frames)")
        print(f"Estimated frames after downsampling: {self.estimated_frames}")

    def extract_first_frame(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise ValueError(f"Could not open video file: {self.video_path}")

        ret, frame = cap.read()
        if not ret:
            raise ValueError("Could not read first frame from video")

        frame_path = os.path.join(self.frames_dir, "00000.jpg")
        cv2.imwrite(frame_path, frame)

        cap.release()
        print(f"First frame saved to {frame_path}")
        return frame, frame_path

    def create_downsampled_video(self):
        print(f"Creating downsampled video at {self.target_fps} FPS...")
        downsampled_path = os.path.join(self.output_dir, "downsampled_video.mp4")

        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise ValueError(f"Could not open video file: {self.video_path}")

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(downsampled_path, fourcc, self.target_fps, (self.width, self.height))

        frame_idx = 0
        saved_count = 0

        with tqdm(total=self.estimated_frames, desc="Creating downsampled video") as pbar:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                if frame_idx % self.frame_interval == 0:
                    out.write(frame)
                    saved_count += 1
                    pbar.update(1)
                frame_idx += 1

        cap.release()
        out.release()

        print(f"Downsampled video saved to {downsampled_path} ({saved_count} frames)")
        return downsampled_path

    def extract_frames(self):
        print(f"Extracting frames at {self.target_fps} FPS...")

        for f in os.listdir(self.frames_dir):
            if f.endswith('.jpg'):
                os.remove(os.path.join(self.frames_dir, f))

        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise ValueError(f"Could not open video file: {self.video_path}")

        frame_paths = []
        frame_idx = 0
        output_idx = 0

        with tqdm(total=self.estimated_frames, desc="Extracting frames") as pbar:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                if frame_idx % self.frame_interval == 0:
                    frame_path = os.path.join(self.frames_dir, f"{output_idx:05d}.jpg")
                    cv2.imwrite(frame_path, frame)
                    frame_paths.append(frame_path)
                    output_idx += 1
                    pbar.update(1)
                frame_idx += 1

        cap.release()
        print(f"Extracted {len(frame_paths)} frames from the video at {self.target_fps} FPS")
        return frame_paths

    def get_multiple_annotations(self):
        first_frame, _ = self.extract_first_frame()
        if first_frame is None or first_frame.size == 0:
            raise ValueError("Failed to load first frame properly")

        while True:
            try:
                num_objects = int(input("\nEnter the number of objects you want to label: "))
                if num_objects <= 0:
                    print("Please enter a positive number.")
                    continue
                break
            except ValueError:
                print("Please enter a valid number.")

        print(f"\nYou will annotate {num_objects} objects.")
        annotations = []

        for obj_idx in range(num_objects):
            print(f"\n--- Annotating object {obj_idx + 1}/{num_objects} ---")
            print("Draw a bounding box on the object by clicking and dragging the mouse.")
            print("Press ENTER when done or ESC to cancel and try again.")

            display_frame = first_frame.copy()
            for ann in annotations:
                x1, y1, x2, y2 = ann["bbox"]
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(display_frame, ann["class_name"], (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            bbox = [0, 0, 0, 0]
            drawing = False

            def mouse_callback(event, x, y, flags, param):
                nonlocal bbox, drawing, temp_img
                if event == cv2.EVENT_LBUTTONDOWN:
                    drawing = True
                    bbox[0], bbox[1] = x, y
                    temp_img[:] = display_frame.copy()
                elif event == cv2.EVENT_MOUSEMOVE:
                    if drawing:
                        temp_img[:] = display_frame.copy()
                        cv2.rectangle(temp_img, (bbox[0], bbox[1]), (x, y), (255, 0, 0), 2)
                elif event == cv2.EVENT_LBUTTONUP:
                    drawing = False
                    bbox[2], bbox[3] = x, y
                    temp_img[:] = display_frame.copy()
                    cv2.rectangle(temp_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)

            window_name = f"Draw bounding box for object {obj_idx + 1}/{num_objects} - Press ENTER when done, ESC to retry"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

            h, w = display_frame.shape[:2]
            display_height = min(800, h)
            display_width = int(w * (display_height / h))
            cv2.resizeWindow(window_name, display_width, display_height)

            temp_img = np.zeros_like(display_frame)
            temp_img[:] = display_frame.copy()
            cv2.setMouseCallback(window_name, mouse_callback)

            while True:
                cv2.imshow(window_name, temp_img)
                key = cv2.waitKey(100) & 0xFF
                if key == 27:  # ESC
                    bbox = [0, 0, 0, 0]
                    temp_img[:] = display_frame.copy()
                    print("Bounding box cleared. Try again.")
                elif key == 13:  # ENTER
                    x1, y1, x2, y2 = bbox
                    if (x1 != x2) and (y1 != y2):
                        break
                    else:
                        print("Invalid bounding box. Please draw a proper box.")

            cv2.destroyAllWindows()

            x1, y1, x2, y2 = bbox
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)
            bbox = [x1, y1, x2, y2]
            print(f"Bounding box: {bbox}")

            class_name = input(f"Enter class name for object {obj_idx + 1}: ")
            if class_name not in self.class_map:
                self.class_map[class_name] = len(self.class_map)

            annotations.append({
                "bbox": bbox,
                "class_name": class_name
            })
            print(f"Added annotation for '{class_name}' with bbox {bbox}")

        print(f"\nCompleted annotation of {num_objects} objects:")
        for idx, ann in enumerate(annotations):
            print(f"{idx + 1}. Class: {ann['class_name']}, Bbox: {ann['bbox']}")
        return annotations

    def segment_with_sam2_multi(self, annotations_data):
        print("\nRunning SAM2 segmentation on downsampled video for multiple objects...")
        downsampled_video = self.create_downsampled_video()

        overrides = dict(conf=0.25, task="segment", mode="predict", imgsz=1024, model=self.sam2_model)
        predictor = SAM2VideoPredictor(overrides=overrides)

        bboxes = [ann["bbox"] for ann in annotations_data]
        class_names = [ann["class_name"] for ann in annotations_data]

        print(f"Processing {len(bboxes)} objects with the following bounding boxes:")
        for i, (bbox, class_name) in enumerate(zip(bboxes, class_names)):
            print(f"{i + 1}. Class: {class_name}, Bbox: {bbox}")

        try:
            results = predictor(source=downsampled_video, bboxes=bboxes)
            print(f"Segmentation completed. Generated {len(results)} frames of annotations.")
            return results, class_names
        except Exception as e:
            print(f"Error during SAM2 segmentation: {e}")
            print(f"Check if the downsampled video was created properly at: {downsampled_video}")
            return [], []

    def results_to_yolo_format_multi(self, results, class_names):
        print("\nConverting SAM2 results to YOLO format for multiple objects...")
        frame_paths = self.extract_frames()
        if len(results) != len(frame_paths):
            print(f"Warning: Number of results ({len(results)}) doesn't match number of frames ({len(frame_paths)})")
            print("Will process only the available frames")

        class_indices = [self.class_map[class_name] for class_name in class_names]
        annotations = {}

        for i, result in enumerate(tqdm(results, desc="Processing results")):
            if i >= len(frame_paths):
                continue
            frame_path = frame_paths[i]

            if not hasattr(result, 'orig_img') or result.orig_img is None:
                print(f"Warning: No original image found in result {i}")
                continue

            orig_img = result.orig_img
            height, width = orig_img.shape[:2]
            frame_annotations = []

            if hasattr(result, 'masks') and result.masks is not None and len(result.masks.data) > 0:
                masks_data = result.masks.data
                for obj_idx, mask in enumerate(masks_data):
                    if obj_idx >= len(class_indices):
                        continue
                    class_idx = class_indices[obj_idx]
                    try:
                        mask_np = mask.cpu().numpy()
                        mask_binary = (mask_np > 0).astype(np.uint8)
                        contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if contours:
                            contour = max(contours, key=cv2.contourArea)
                            x, y, w, h = cv2.boundingRect(contour)
                            x_center = (x + w/2) / width
                            y_center = (y + h/2) / height
                            bbox_width = w / width
                            bbox_height = h / height
                            frame_annotations.append({
                                'class_idx': class_idx,
                                'bbox': [x_center, y_center, bbox_width, bbox_height]
                            })
                    except Exception as e:
                        print(f"Error processing mask for object {obj_idx}: {str(e)}")
                        continue

            elif hasattr(result, 'boxes') and result.boxes is not None and len(result.boxes.data) > 0:
                boxes_data = result.boxes.data
                for obj_idx, box_data in enumerate(boxes_data):
                    if obj_idx >= len(class_indices):
                        continue
                    class_idx = class_indices[obj_idx]
                    x1, y1, x2, y2, conf, _ = box_data.cpu().numpy()
                    x_center = (x1 + x2) / (2 * width)
                    y_center = (y1 + y2) / (2 * height)
                    bbox_width = (x2 - x1) / width
                    bbox_height = (y2 - y1) / height
                    frame_annotations.append({
                        'class_idx': class_idx,
                        'bbox': [x_center, y_center, bbox_width, bbox_height],
                        'conf': float(conf)
                    })

            if frame_annotations:
                annotations[frame_path] = frame_annotations

        if not annotations:
            print("Warning: No valid annotations were generated!")
        else:
            print(f"Successfully created annotations for {len(annotations)} frames")
        return annotations, frame_paths

    def create_yolo_dataset(self, all_annotations, frame_paths, train_ratio=0.8):
        print("\nCreating YOLO dataset...")
        annotated_frames = list(all_annotations.keys())
        if not annotated_frames:
            print("No valid annotations found. Cannot create dataset.")
            return None

        random.shuffle(annotated_frames)
        train_count = int(len(annotated_frames) * train_ratio)
        train_frames = annotated_frames[:train_count]
        val_frames = annotated_frames[train_count:]

        print(f"Creating dataset with {len(train_frames)} training and {len(val_frames)} validation frames")

        for frame_path in tqdm(train_frames, desc="Processing training data"):
            if frame_path in all_annotations:
                self._save_yolo_data(frame_path, all_annotations[frame_path], "train")

        for frame_path in tqdm(val_frames, desc="Processing validation data"):
            if frame_path in all_annotations:
                self._save_yolo_data(frame_path, all_annotations[frame_path], "val")

        yaml_path = os.path.join(self.output_dir, "dataset.yaml")
        with open(yaml_path, "w") as f:
            f.write(f"path: {os.path.abspath(self.dataset_dir)}\n")
            f.write("train: train/images\n")
            f.write("val: val/images\n")
            f.write("names:\n")
            for class_name, idx in sorted(self.class_map.items(), key=lambda x: x[1]):
                f.write(f"  {idx}: {class_name}\n")

        train_images = len(os.listdir(os.path.join(self.dataset_dir, "train", "images")))
        val_images = len(os.listdir(os.path.join(self.dataset_dir, "val", "images")))

        print(f"\nDataset Statistics:")
        print(f"Total frames with annotations: {len(annotated_frames)}")
        print(f"Training images: {train_images}")
        print(f"Validation images: {val_images}")
        print(f"Classes: {', '.join(self.class_map.keys())}")
        return yaml_path

    def _save_yolo_data(self, frame_path, annotations, split):
        frame_name = os.path.basename(frame_path)
        label_name = os.path.splitext(frame_name)[0] + ".txt"
        img_dest_path = os.path.join(self.dataset_dir, split, "images", frame_name)
        label_path = os.path.join(self.dataset_dir, split, "labels", label_name)
        shutil.copy(frame_path, img_dest_path)
        with open(label_path, "w") as f:
            for ann in annotations:
                bbox_str = " ".join([f"{x:.6f}" for x in ann['bbox']])
                f.write(f"{ann['class_idx']} {bbox_str}\n")

    def generate_training_data_multi(self, train_ratio=0.8):
        try:
            annotations_data = self.get_multiple_annotations()
            results, class_names = self.segment_with_sam2_multi(annotations_data)
            if not results:
                print("SAM2 segmentation failed. Please try again.")
                return None
            annotations, frame_paths = self.results_to_yolo_format_multi(results, class_names)
            if not annotations:
                print("No valid annotations were created. Please try again.")
                return None
            yaml_path = self.create_yolo_dataset(annotations, frame_paths, train_ratio)
            if yaml_path:
                print("\nTraining data generation completed!")
                print(f"Dataset created at: {self.dataset_dir}")
                print(f"Configuration file: {yaml_path}")
                return yaml_path
            else:
                print("\nFailed to create dataset.")
                return None
        except Exception as e:
            print(f"Error in data generation: {e}")
            import traceback
            traceback.print_exc()
            return None

    @staticmethod
    def dataset_exists(output_dir):
        """Check if a dataset already exists in the given output directory."""
        return os.path.exists(os.path.join(output_dir, "dataset.yaml"))

class SAM2YOLOGeneratorNode(Node):
    def __init__(self):
        super().__init__('sam2_yolo_generator')
        self.declare_parameter('video_folder', rclpy.Parameter.Type.STRING)
        self.declare_parameter('output_base_dir', '')
        self.declare_parameter('sam2_model', 'sam2.1_b.pt')
        self.declare_parameter('target_fps', 7)
        self.declare_parameter('train_ratio', 0.8)
        self.declare_parameter('skip_existing', True)
        self.declare_parameter('force_reprocess', False)

        video_folder = self.get_parameter('video_folder').value
        output_base_dir = self.get_parameter('output_base_dir').value
        sam2_model = self.get_parameter('sam2_model').value
        target_fps = self.get_parameter('target_fps').value
        train_ratio = self.get_parameter('train_ratio').value
        skip_existing = self.get_parameter('skip_existing').value
        force_reprocess = self.get_parameter('force_reprocess').value

        # If output_base_dir is empty, use a default inside the package share directory
        if not output_base_dir:
            try:
                package_share = get_package_share_directory('your_package_name')  # Change to actual package name
                output_base_dir = os.path.join(package_share, 'datasets')
            except Exception as e:
                self.get_logger().error(f"Could not find package share directory: {e}")
                output_base_dir = os.path.expanduser('~/sam2_yolo_datasets')
                self.get_logger().warn(f"Using fallback output directory: {output_base_dir}")

        if not video_folder:
            self.get_logger().error("Parameter 'video_folder' must be provided.")
            rclpy.shutdown()
            return

        if not os.path.isdir(video_folder):
            self.get_logger().error(f"Video folder does not exist: {video_folder}")
            rclpy.shutdown()
            return

        # Create output base directory
        os.makedirs(output_base_dir, exist_ok=True)

        # Find video files
        video_extensions = ('.mp4', '.avi', '.mov', '.mkv')
        video_files = [f for f in os.listdir(video_folder) if f.lower().endswith(video_extensions)]
        if not video_files:
            self.get_logger().error(f"No video files found in {video_folder}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Found {len(video_files)} video(s) in {video_folder}")
        for vf in video_files:
            self.get_logger().info(f"  {vf}")

        # Process each video
        for video_file in video_files:
            video_path = os.path.join(video_folder, video_file)
            video_name = os.path.splitext(video_file)[0]
            output_dir = os.path.join(output_base_dir, video_name)
            self.get_logger().info(f"Processing video: {video_path} -> {output_dir}")

            # Check if dataset already exists
            if skip_existing and SAM2YOLOGenerator.dataset_exists(output_dir) and not force_reprocess:
                self.get_logger().info(f"Dataset already exists at {output_dir}. Skipping (use force_reprocess=True to override).")
                continue

            # Run generator
            generator = SAM2YOLOGenerator(
                video_path=video_path,
                output_dir=output_dir,
                sam2_model=sam2_model,
                target_fps=target_fps
            )
            generator.generate_training_data_multi(train_ratio=train_ratio)

        self.get_logger().info("All videos processed.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SAM2YOLOGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()