import rclpy
from rclpy.node import Node
from anafi_msgs.msg import CurrentState,DroneSize, KeyPoints
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import threading
import time
import logging
from geometry_msgs.msg import Vector3
import csv
from ultralytics import YOLO
from rclpy.executors import MultiThreadedExecutor
import copy


logging.getLogger("olympe").setLevel(logging.CRITICAL)


class GetKeyPoints(Node):
    def __init__(self):
        super().__init__('get_keypoints')

        self.root_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws')
        self.model_path = os.path.join(self.root_dir, "src", "track_parrot","train_drone_yolo_2d","drone_detection","weights","best.pt")

        self.model = YOLO(self.model_path)

        self.save_dir_img = os.path.join(os.path.expanduser("~"), 'anafi_ws','data','keypoints','img')
        self.save_dir_keypoints = os.path.join(os.path.expanduser("~"), 'anafi_ws','data','keypoints','keypoint')
        os.makedirs(self.save_dir_img, exist_ok=True)
        os.makedirs(self.save_dir_keypoints, exist_ok=True)

        self.file_path = os.path.join(self.save_dir_keypoints, 'keypoints.csv')

        self.bridge = CvBridge()    

        self.target_frame_anafi = 'anafi'
        self.target_frame_parrot = 'bebop1'
        self.frameid = 0

        self.running = True
        self.connected = False
        self.flag = False

        self.drone_state_anafi = CurrentState()
        self.drone_state_parrot = CurrentState()
        self.image = None

        self.show_img = None

        self.image_id = 0

        self.drone_size = DroneSize()
        # self.drone_size.length = 0.430
        # self.drone_size.width = 0.508
        # self.drone_size.height = 0.300

        

        self.camera_angle = Vector3()
        
        self.keypoints_anafi_frame_2d = KeyPoints()

        self.keypoints_name = [
                "front_top_left",
                "front_top_right",
                "front_bottom_left",
                "front_bottom_right",
                "rear_top_left",
                "rear_top_right",
                "rear_bottom_left",
                "rear_bottom_right"
            ]

       
        self.image_sub = self.create_subscription(Image, '/anafi/vedio', self.image_sub_callback, 10)   
        self.keypoints_anafi_frame_2d_sub = self.create_subscription(KeyPoints, '/keypoints_2d', self.keypoints_anafi_frame_2d_sub_callback, 10)

        self.init_csv_file()

        # self.start_user_input_thread()

        self.show_img_thread = threading.Thread(target = self.show_img_thread_callback)
        self.show_img_thread.daemon = True
        self.show_img_thread.start()

        self.get_keypoints_thread = threading.Thread(target = self.get_keypoints_thread_callback)
        self.get_keypoints_thread.daemon = True
        self.get_keypoints_thread.start()

    def init_csv_file(self):
        with open(self.file_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            header = ["Timestamp"] + [f"{kp}_{coord}" for kp in self.keypoints_name for coord in ["x", "y"]]
            writer.writerow(header)



    def get_keypoints_thread_callback(self):
        def draw_keypoints_cube(image, keypoints_list):
            height, width, _ = image.shape
    
            edges = [
                (0, 1), (2, 3), (0, 2), (1, 3),  # Front face
                (4, 5), (6, 7), (4, 6), (5, 7),  # Back face
                (0, 4), (1, 5), (2, 6), (3, 7)   # Connecting edges between front and back faces
            ]

            for edge in edges:
                pt1 = keypoints_list[edge[0]]
                pt2 = keypoints_list[edge[1]]

                if (0 <= pt1[0] < width and 0 <= pt1[1] < height) and (0 <= pt2[0] < width and 0 <= pt2[1] < height):
                    self.flag = True
                    
                else:
                    self.flag = False
                    
            if self.flag == False:      
                return image
            
            else:
                for edge in edges:
                    pt1 = keypoints_list[edge[0]]
                    pt2 = keypoints_list[edge[1]]
                    cv2.line(image, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (0, 255, 0), 2)
            
            return image


        while self.running:

            sum_x = 0
            sum_y = 0

            if self.image is not None:

                current_image = copy.deepcopy(self.image)  # Use a copy to avoid race conditions
                save_image = copy.deepcopy(self.image)

                keypoints_anafi_frame_2d_current = self.keypoints_anafi_frame_2d
                self.get_logger().info(f"Keypoints data received: {keypoints_anafi_frame_2d_current}")
                
                height, width, _ = current_image.shape
                self.get_logger().debug(f"Image dimensions: {height}x{width}")

                for keypoint_name in self.keypoints_name:
                    keypoint = getattr(keypoints_anafi_frame_2d_current, keypoint_name)
                    sum_x += keypoint.x
                    sum_y += keypoint.y
                
                average_x = int(sum_x / 8)
                average_y = int(sum_y / 8)

                results = self.model(current_image)
                # Draw bounding boxes and refine results
                highest_confidence = -1
                best_box = None
                best_x_center, best_y_center = None, None

                for result in results:
                    if result.boxes:
                        for box in result.boxes:
                            # Extract bounding box coordinates
                            x1, y1, x2, y2 = map(int, box.xyxy[0])

                            x_center = (x1+x2)/2
                            y_center = (y1+y2)/2

                            cls = int(box.cls[0])
                            confidence = float(box.conf[0])
                            label = self.model.names[cls]

                            if label.lower() == 'drone' and confidence >= 0.5:
                                if confidence > highest_confidence:
                                    highest_confidence = confidence
                                    best_box = (x1, y1, x2, y2)
                                    best_x_center = x_center
                                    best_y_center = y_center
                                    width = x2 - x1 
                                    height = y2 - y1

                if best_box:
                    delta_x = best_x_center - average_x
                    delta_y = best_y_center - average_y

                    self.get_logger().info(f"Best X Center: {best_x_center}, Best Y Center: {best_y_center}")

                
                    keypoints_list = []
                    for keypoint_name in self.keypoints_name:
                        keypoint = getattr(keypoints_anafi_frame_2d_current, keypoint_name)
                        keypoint.x += delta_x
                        keypoint.y += delta_y
                        keypoints_list.append([keypoint.x, keypoint.y])
                    
                    image_with_cube = draw_keypoints_cube(current_image, keypoints_list)
                    cv2.imshow("Image with keypoints cube", image_with_cube)
                    cv2.waitKey(1)

                    if self.image_id <= 999:            
                        image_save_path = os.path.join(self.save_dir_img, f'{self.image_id}.jpg')
                        cv2.imwrite(image_save_path, save_image)
                        with open(self.file_path, mode="a", newline="") as file:
                                writer = csv.writer(file)
                                # Flatten the keypoints_list
                                bbox = [best_x_center, best_y_center, width, height]
                                flattened_keypoints = [coord for keypoint in keypoints_list for coord in keypoint]
                                combined_row = bbox + flattened_keypoints
                                writer.writerow(combined_row ) 

                    self.image_id += 1

                    
                
                else:
                    image_with_cube = current_image
                    cv2.imshow("Image with keypoints cube", image_with_cube)
                    cv2.waitKey(1)
                

            else:
                self.get_logger().info('Wait for image!')
            
            time.sleep(0.1)
        


    def image_sub_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        
    
    def keypoints_anafi_frame_2d_sub_callback(self, msg):
        self.keypoints_anafi_frame_2d = msg

        
        

    def show_img_thread_callback(self):
        while self.running:
            if self.show_img is not None:
                cv2.imshow("Image with keypoints cube", self.show_img)
                cv2.waitKey(1)
            time.sleep(0.1)
        pass

    def Stop(self):
        self.running = False
        self.get_logger().info('Shutting down...\n')
        self.Connected = False
        time.sleep(0.2)
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GetKeyPoints()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        # Spin the executor
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        # Cleanup
        executor.shutdown()
        node.Stop()  # Custom method to stop your node's operations
        rclpy.shutdown()

if __name__ == '__main__':
    main()