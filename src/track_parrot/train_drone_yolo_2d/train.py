from ultralytics import YOLO
import os
import torch


def prepare_and_train_yolo():
    """
    Prepares YOLOv8 for training using the labeled drone dataset and trains the model.
    """
    # Define paths
    script_dir = os.path.dirname(os.path.abspath(__file__))  # folder path
    data_yaml_path = os.path.join(script_dir, 'data.yaml')   # file in that folder

    model_type = "yolov8n.pt"  # Use 'yolov8n.pt' (nano), 'yolov8s.pt' (small), etc.

    # Check GPU availability
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    # Check if data.yaml exists
    if not os.path.exists(data_yaml_path):
        raise FileNotFoundError(f"{data_yaml_path} not found. Please create a YAML file for your dataset.")

    # Load a pre-trained YOLOv8 model
    print("Loading pre-trained YOLO model...")
    model = YOLO(model_type)

    # Train the model
    print("Starting training...")
    model.train(
        data=data_yaml_path,  # Path to dataset configuration file
        epochs=50,  # Number of epochs
        imgsz=640,  # Image size
        batch=16,  # Batch size
        workers=4,  # Number of dataloader workers
        device=device,  # Specify the device ('cuda' for GPU, 'cpu' for CPU)
        project="src/track_parrot/train_drone_yolo_2d",  # Directory to save results
        name="drone_detection",  # Experiment name
    )

    print("Training complete. Best weights saved in runs/detect/drone_detection/weights/best.pt")





if __name__ == "__main__":
    # Uncomment one of the following functions based on your workflow:
    
    # 1. Train the YOLO model
    prepare_and_train_yolo()


 
