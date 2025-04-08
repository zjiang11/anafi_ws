from ultralytics import YOLO
import os
import torch





def test_inference(script_dir, input_dir, output_dir):
    """
    Tests inference using the trained YOLO model on new images and calculates the average confidence score.
    """
    import os
    import torch
    from ultralytics import YOLO

    model_path = os.path.join(script_dir, 'drone_detection/weights/best.pt')

    os.makedirs(output_dir, exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    total_confidence = 0
    total_detections = 0

    for filename in os.listdir(input_dir):
        # Check if the file is an image
        if filename.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.tiff')):
            input_path = os.path.join(input_dir, filename)
            output_path = os.path.join(output_dir, filename)

            print(f"Using device: {device}")
            print("Loading trained model for inference...")
            model = YOLO(model_path)

            print(f"Running inference on {input_path}...")
            results = model(input_path, device=device)

            print("Inference complete. Showing results...")
            for result in results:
                result.save(filename=output_path)  # Save output to the specified path

                # Extract confidence scores safely
                if result.boxes is not None and len(result.boxes) > 0:
                    confidences = result.boxes.conf.tolist()  # Use .conf to access confidence directly
                    total_confidence += sum(confidences)
                    total_detections += len(confidences)

            print(f"Saved result to {output_path}")

    # Calculate average confidence score
    if total_detections > 0:
        average_confidence = total_confidence / total_detections
        print(f"\nAverage Confidence Score: {average_confidence:.4f}")
    else:
        print("\nNo detections found.")



if __name__ == "__main__":



    # 3. Run inference with the YOLO model
    script_dir = os.path.dirname(os.path.abspath(__file__))

    input_dir = os.path.join(script_dir, 'parrot_test_figures')  # Path to input directory
    output_dir = os.path.join(script_dir, 'parrot_test_results')  # Path to output directory
    
    test_inference(script_dir, input_dir, output_dir)
