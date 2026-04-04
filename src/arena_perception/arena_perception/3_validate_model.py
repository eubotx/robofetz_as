import cv2
import argparse
import time
import torch
import numpy as np
from pathlib import Path
from ultralytics import YOLO


def run_live_demo(model_path, video_path, conf_threshold=0.25, save_output=True):
    """
    Run live prediction demo on a video file using a trained YOLO model
    
    Args:
        model_path (str): Path to the trained YOLO model (.pt file)
        video_path (str): Path to the input video file
        conf_threshold (float): Confidence threshold for detections
        save_output (bool): Whether to save the output video
    """
    # Load the model
    print(f"Loading model from {model_path}")
    model = YOLO(model_path)
    
    # Get class names
    class_names = model.names
    print(f"Loaded model with {len(class_names)} classes: {class_names}")
    
    # Open the video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError(f"Could not open video: {video_path}")
    
    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"Video properties: {width}x{height} at {fps} FPS, {total_frames} frames")
    
    # Create video writer if saving output
    if save_output:
        output_path = f"{Path(video_path).stem}_yolo_demo.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        print(f"Saving output to {output_path}")
    else:
        out = None
    
    # Create window for display
    cv2.namedWindow("YOLO Detection Demo", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("YOLO Detection Demo", min(1280, width), min(720, height))
    
    # Performance tracking
    frame_times = []
    processing_times = []
    total_detections = 0
    
    # Start processing
    frame_count = 0
    print("Starting video processing. Press 'q' to quit, 's' to save a screenshot.")
    
    while True:
        # Measure frame time
        frame_start = time.time()
        
        # Read frame
        ret, frame = cap.read()
        if not ret:
            break
        
        # Measure processing time
        process_start = time.time()
        
        # Run inference
        results = model(frame, conf=conf_threshold, verbose=False)[0]
        
        # Track processing time
        process_end = time.time()
        processing_time = process_end - process_start
        processing_times.append(processing_time)
        
        # Count detections
        num_detections = len(results.boxes)
        total_detections += num_detections
        
        # Draw bounding boxes and labels
        # Create a copy of the frame to avoid modifying the original
        annotated_frame = frame.copy()
        
        # Process each detection
        for box in results.boxes:
            # Extract bbox coordinates
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            
            # Extract confidence and class
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = class_names.get(cls_id, f"Class {cls_id}")
            
            # Generate color for this class (different color for each class)
            color_factor = cls_id / len(class_names) if len(class_names) > 0 else 0
            color = tuple(int(c) for c in (120 + 120 * np.sin(color_factor * 2 * np.pi), 
                                          120 + 120 * np.sin(color_factor * 2 * np.pi + 2*np.pi/3), 
                                          120 + 120 * np.sin(color_factor * 2 * np.pi + 4*np.pi/3)))
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            
            # Create label with class name and confidence
            label = f"{cls_name}: {conf:.2f}"
            
            # Get size of text for background rectangle
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            
            # Draw background rectangle for text
            cv2.rectangle(annotated_frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1)
            
            # Draw text
            cv2.putText(annotated_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Add frame info (FPS, frame number, detections)
        frame_time = time.time() - frame_start
        frame_times.append(frame_time)
        current_fps = 1 / (sum(frame_times[-20:]) / min(len(frame_times), 20)) if frame_times else 0
        
        info_text = f"Frame: {frame_count}/{total_frames} | FPS: {current_fps:.1f} | Detections: {num_detections}"
        cv2.putText(annotated_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display the annotated frame
        cv2.imshow("YOLO Detection Demo", annotated_frame)
        
        # Write to output video if saving
        if out is not None:
            out.write(annotated_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # Quit on 'q'
            print("Quitting...")
            break
        elif key == ord('s'):
            # Save screenshot on 's'
            screenshot_path = f"screenshot_{frame_count:04d}.jpg"
            cv2.imwrite(screenshot_path, annotated_frame)
            print(f"Saved screenshot to {screenshot_path}")
        
        frame_count += 1
        
        # Print progress every 100 frames
        if frame_count % 100 == 0:
            progress = frame_count / total_frames * 100
            avg_fps = frame_count / sum(frame_times)
            print(f"Progress: {progress:.1f}% | Frame: {frame_count}/{total_frames} | Avg FPS: {avg_fps:.1f}")
    
    # Clean up
    cap.release()
    if out is not None:
        out.release()
    cv2.destroyAllWindows()
    
    # Print summary
    if frame_count > 0:
        avg_process_time = sum(processing_times) / len(processing_times)
        avg_fps = frame_count / sum(frame_times)
        avg_detections = total_detections / frame_count
        
        print("\n--- Performance Summary ---")
        print(f"Processed {frame_count} frames at {avg_fps:.1f} FPS")
        print(f"Average processing time: {avg_process_time*1000:.1f} ms per frame")
        print(f"Average detections: {avg_detections:.1f} per frame")
        print(f"Total detections: {total_detections}")
        
        if save_output:
            print(f"Output video saved to: {output_path}")


def batch_process_frames(model_path, video_path, output_dir="frames_with_predictions", 
                        frame_interval=10, conf_threshold=0.25):
    """
    Process frames from a video at specified intervals and save them with predictions
    
    Args:
        model_path (str): Path to the trained YOLO model
        video_path (str): Path to the input video
        output_dir (str): Directory to save output frames
        frame_interval (int): Process every Nth frame
        conf_threshold (float): Confidence threshold for detections
    """
    import os
    from tqdm import tqdm
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Load model
    print(f"Loading model from {model_path}")
    model = YOLO(model_path)
    
    # Open video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError(f"Could not open video: {video_path}")
    
    # Get video properties
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"Video has {total_frames} frames at {fps} FPS")
    print(f"Processing every {frame_interval}th frame")
    
    # Expected frames to process
    expected_frames = total_frames // frame_interval
    
    # Process frames
    frame_idx = 0
    saved_count = 0
    
    with tqdm(total=expected_frames, desc="Processing frames") as pbar:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Process every Nth frame
            if frame_idx % frame_interval == 0:
                # Run inference
                results = model(frame, conf=conf_threshold, verbose=False)[0]
                
                # Draw results on the frame
                annotated_frame = results.plot()
                
                # Add frame info
                time_info = f"Time: {frame_idx/fps:.2f}s (Frame {frame_idx})"
                cv2.putText(annotated_frame, time_info, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Save the frame
                output_path = os.path.join(output_dir, f"frame_{frame_idx:05d}.jpg")
                cv2.imwrite(output_path, annotated_frame)
                
                saved_count += 1
                pbar.update(1)
            
            frame_idx += 1
    
    cap.release()
    print(f"Processed {frame_idx} frames, saved {saved_count} frames to {output_dir}")


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Run live demo with trained YOLO model")
    parser.add_argument("--model", type=str, required=True, help="Path to trained YOLO model (.pt file)")
    parser.add_argument("--video", type=str, required=True, help="Path to input video file")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--no-save", action="store_true", help="Don't save output video")
    parser.add_argument("--batch", action="store_true", help="Run in batch mode (save frames)")
    parser.add_argument("--interval", type=int, default=10, help="Frame interval for batch mode")
    
    return parser.parse_args()


def main():
    """Main function"""
    args = parse_args()
    
    if args.batch:
        # Run in batch mode
        batch_process_frames(
            model_path=args.model,
            video_path=args.video,
            frame_interval=args.interval,
            conf_threshold=args.conf
        )
    else:
        # Run live demo
        run_live_demo(
            model_path=args.model,
            video_path=args.video,
            conf_threshold=args.conf,
            save_output=not args.no_save
        )


if __name__ == "__main__":
    # If no arguments are provided, use these defaults
    import sys
    if len(sys.argv) == 1:
        # Example default settings
        sys.argv.extend([
            "--model", "/home/sjadav/Documents/Projects/ARL_TOH/YOLO_TRAIN/yolo_results/train/weights/best.pt",  # Path to your trained model
            "--video", "/home/sjadav/Documents/Projects/ARL_TOH/YOLO_TRAIN/blocks.mp4",                  # Path to your original video
            "--conf", "0.8"                                   # Confidence threshold
        ])
    
    main()