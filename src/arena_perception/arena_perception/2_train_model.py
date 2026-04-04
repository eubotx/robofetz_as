import os
import argparse
import yaml
import matplotlib.pyplot as plt
import cv2
import numpy as np
from pathlib import Path
from ultralytics import YOLO
from tqdm import tqdm


class YOLOTrainer:
    def __init__(self, 
                 dataset_yaml, 
                 model_type="yolov8n.pt", 
                 epochs=100, 
                 batch_size=16, 
                 imgsz=640,
                 output_dir="yolo_results"):
        """
        Initialize the YOLO trainer
        
        Args:
            dataset_yaml (str): Path to the dataset YAML file
            model_type (str): YOLO model type or path to pretrained weights
            epochs (int): Number of training epochs
            batch_size (int): Batch size for training
            imgsz (int): Image size for training
            output_dir (str): Directory to save results
        """
        self.dataset_yaml = dataset_yaml
        self.model_type = model_type
        self.epochs = epochs
        self.batch_size = batch_size
        self.imgsz = imgsz
        self.output_dir = output_dir
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Load dataset configuration to get class names
        with open(dataset_yaml, 'r') as f:
            self.dataset_config = yaml.safe_load(f)
        
        # Initialize model
        print(f"Loading model: {model_type}")
        self.model = YOLO(model_type)
    
    def train(self):
        """Train the YOLO model on the dataset"""
        print(f"\n--- Starting YOLO training for {self.epochs} epochs ---")
        print(f"Dataset: {self.dataset_yaml}")
        print(f"Image size: {self.imgsz}")
        print(f"Batch size: {self.batch_size}")
        
        # Train the model with specified parameters
        results = self.model.train(
            data=self.dataset_yaml,
            epochs=self.epochs,
            batch=self.batch_size,
            imgsz=self.imgsz,
            project=self.output_dir,
            name="train",
            exist_ok=True,
            pretrained=True,
            plots=True,
            save=True
        )
        
        # Save training results
        print(f"\nTraining completed. Results saved to {self.output_dir}/train")
        return results
    
    def validate(self):
        """Validate the trained model on the validation set"""
        print("\n--- Running validation on the trained model ---")
        
        # Get the best model path
        best_weights = Path(self.output_dir) / "train" / "weights" / "best.pt"
        if not best_weights.exists():
            print(f"Best weights not found at {best_weights}. Using last weights.")
            best_weights = Path(self.output_dir) / "train" / "weights" / "last.pt"
        
        # Load the trained model
        print(f"Loading trained model: {best_weights}")
        trained_model = YOLO(best_weights)
        
        # Get validation path from dataset YAML
        val_path = Path(self.dataset_config['path']) / self.dataset_config['val']
        print(f"Validation dataset: {val_path}")
        
        # Run validation
        val_results = trained_model.val(
            data=self.dataset_yaml,
            split="val",
            project=self.output_dir,
            name="validation",
            exist_ok=True,
            plots=True
        )
        
        print("\nValidation metrics:")
        print(f"mAP50-95: {val_results.box.map}")
        print(f"mAP50: {val_results.box.map50}")
        print(f"Precision: {val_results.box.p}")
        print(f"Recall: {val_results.box.r}")
        
        return val_results
    
    def visualize_predictions(self, num_images=10, conf_threshold=0.25, save_dir=None):
        """
        Visualize model predictions on validation images
        
        Args:
            num_images (int): Number of images to visualize
            conf_threshold (float): Confidence threshold for predictions
            save_dir (str): Directory to save visualization results
        """
        print(f"\n--- Visualizing predictions on {num_images} validation images ---")
        
        # Use the best model for predictions
        best_weights = Path(self.output_dir) / "train" / "weights" / "best.pt"
        if not best_weights.exists():
            print(f"Best weights not found at {best_weights}. Using last weights.")
            best_weights = Path(self.output_dir) / "train" / "weights" / "last.pt"
        
        # Load the trained model
        model = YOLO(best_weights)
        
        # Get validation image paths
        val_path = Path(self.dataset_config['path']) / self.dataset_config['val'] / "images"
        image_files = list(val_path.glob("*.jpg")) + list(val_path.glob("*.png"))
        
        if not image_files:
            print(f"No images found in {val_path}")
            return
        
        # Limit the number of images
        if len(image_files) > num_images:
            image_files = image_files[:num_images]
        
        # Create save directory if specified
        if save_dir is None:
            save_dir = Path(self.output_dir) / "visualizations"
        os.makedirs(save_dir, exist_ok=True)
        
        # Get class names from dataset config
        class_names = self.dataset_config.get('names', {})
        
        # Process each image
        print(f"Processing {len(image_files)} images...")
        
        for img_path in tqdm(image_files):
            # Run prediction
            results = model(img_path, conf=conf_threshold)[0]
            
            # Read original image
            img = cv2.imread(str(img_path))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # Plot predictions on the image
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                # Get class name
                cls_name = class_names.get(cls_id, f"Class {cls_id}")
                
                # Draw bounding box
                color = (0, 255, 0)  # Green color for bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                
                # Add label with confidence
                label = f"{cls_name}: {conf:.2f}"
                font_scale = 0.6
                font_thickness = 1
                (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
                
                # Draw background rectangle for text
                cv2.rectangle(img, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1)
                
                # Draw text
                cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness)
            
            # Save the image with predictions
            out_path = os.path.join(save_dir, img_path.name)
            plt.figure(figsize=(12, 8))
            plt.imshow(img)
            plt.axis('off')
            plt.title(f"Predictions: {img_path.name}")
            plt.tight_layout()
            plt.savefig(out_path)
            plt.close()
        
        print(f"Visualizations saved to {save_dir}")
    
    def export_model(self, format="onnx"):
        """
        Export the trained model to different formats
        
        Args:
            format (str): Export format (onnx, torchscript, openvino, etc.)
        """
        print(f"\n--- Exporting model to {format} format ---")
        
        # Use the best model for export
        best_weights = Path(self.output_dir) / "train" / "weights" / "best.pt"
        if not best_weights.exists():
            print(f"Best weights not found at {best_weights}. Using last weights.")
            best_weights = Path(self.output_dir) / "train" / "weights" / "last.pt"
        
        # Load the trained model
        model = YOLO(best_weights)
        
        # Export the model
        export_path = model.export(format=format)
        
        print(f"Model exported to: {export_path}")
        return export_path


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Train custom YOLO model on SAM2 generated dataset")
    parser.add_argument("--data", type=str, required=True, help="Path to dataset YAML file")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="YOLO model type or path to pretrained weights")
    parser.add_argument("--epochs", type=int, default=100, help="Number of training epochs")
    parser.add_argument("--batch", type=int, default=16, help="Batch size for training")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size for training")
    parser.add_argument("--output", type=str, default="yolo_results", help="Directory to save results")
    parser.add_argument("--visualize", type=int, default=10, help="Number of validation images to visualize (0 to skip)")
    parser.add_argument("--export", action="store_true", help="Export model after training")
    parser.add_argument("--export-format", type=str, default="onnx", help="Format to export the model")
    
    return parser.parse_args()


def main():
    """Main function"""
    args = parse_args()
    
    # Create YOLO trainer
    trainer = YOLOTrainer(
        dataset_yaml=args.data,
        model_type=args.model,
        epochs=args.epochs,
        batch_size=args.batch,
        imgsz=args.imgsz,
        output_dir=args.output
    )
    
    # Train the model
    trainer.train()
    
    # Validate the model
    trainer.validate()
    
    # Visualize predictions on validation images
    if args.visualize > 0:
        trainer.visualize_predictions(num_images=args.visualize)
    
    # Export the model if requested
    if args.export:
        trainer.export_model(format=args.export_format)
    
    print("\nTraining and evaluation completed!")


# Example usage (also works for command line)
if __name__ == "__main__":
    # If no arguments are provided, use these defaults
    import sys
    if len(sys.argv) == 1:
        # Example default settings
        sys.argv.extend([
            "--data", "/home/sjadav/Documents/Projects/ARL_TOH/YOLO_TRAIN/dataset_blocks_statics/dataset.yaml",  # Path to your dataset YAML
            "--model", "yolov8m.pt",           # Using YOLOv8 small model
            "--epochs", "30",                  # Train for 50 epochs
            "--batch", "8",                    # Batch size
            "--imgsz", "640",                  # Image size
            "--visualize", "5",                # Visualize 5 validation images
            "--export"                         # Export model after training
        ])
    
    main()