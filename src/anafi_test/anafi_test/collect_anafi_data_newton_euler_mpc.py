import rclpy
import olympe
from rclpy.node import Node
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
import time
import os
from anafi_msgs.msg import Output, CurrentState, CollectCurrentState
import threading
import csv
from pynput.keyboard import Listener, Key
import logging
import math
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import transforms3d



logging.getLogger("olympe").setLevel(logging.CRITICAL)

class CollectDataNode(Node):
    def __init__(self):
        super().__init__('collect_data_node')
        self.connected = False
        self.running = True
        self.write_data_flag = False
        self.freq = 25
        self.drone = None
        self.is_test_on = False
        self.test_axis = None        
        self.csv_file = None  # Initialize csv_file as None
        self.connected = False
        self.is_control_on = False
        self.is_manual_on = True
        self.previous_time_update = True

        self.target_frame = 'anafi'
        self.cumulative_time_stamp = 0.0
        self.time_stamp = 0.0
        self.control_output = Output()
        self.collect_current_state = CollectCurrentState()
        self.pcmd = Output()
        self.drone_state = CurrentState()

        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.gaz = 0
        self.takeoff = False
        self.land = False

        # MANUAL MODE
        self.x_manual = 0
        self.y_manual = 0
        self.z_manual = 0
        self.yaw_manual = 0

        self.x_control = 0
        self.y_control = 0
        self.z_control = 0
        self.yaw_control = 0

        self.x_error = 0
        self.y_error = 0
        self.z_error = 0
        self.yaw_error = 0

        self.freq_publish_pcmd = 25

        self.previous_time = None
        self.previous_x = None
        self.previous_y = None
        self.previous_z = None
        self.previous_roll = None
        self.previous_pitch = None
        self.previous_yaw = None

        self.window_size = 100
        self.cutoff_freq_x = 2
        self.cutoff_freq_y = 2
        self.cutoff_freq_z = 2
        self.cutoff_freq_yaw = 2

        self.position_window_x = []
        self.position_window_y = []
        self.position_window_z = []
        self.position_window_yaw = []

        self.time_window = []

        self.timer_pubPCMD = self.create_timer(callback=self.publish_pcmd, timer_period_sec=1/self.freq_publish_pcmd)
        self.calculate_speed = self.create_timer(callback= self.calculate_speed_callback, timer_period_sec=1/self.freq)
        self.subscribe_drone_tf = self.create_subscription(TFMessage, 'tf',self.drone_tf_callback, 10)
        self.publisher = self.create_publisher(CollectCurrentState, '/drone_current_state', 10)
        self.pcmd_publisher = self.create_publisher(Output, '/output', 10)
        self.publish_anafi_state = self.create_publisher(CollectCurrentState, 'anafi_state', 10)

        self.data_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data', 'anafi_state_function', 'newton_euler_mpc')
        os.makedirs(self.data_dir, exist_ok=True) 
        self.start_user_input_thread()


        save_data_thread = threading.Thread(target=self.save_data_thread_callback)
        save_data_thread.daemon = True
        save_data_thread.start()
        


        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        if hasattr(key, 'char') and (key.char == 'w' or key.char == 's' or key.char == 'a' or key.char == 'd' or key.char == 'c' or key.char == 'x' or key.char == 'f' or key.char == 'r'): 
           self.is_control_on = False
           self.is_manual_on = True

        if key == Key.left:
            self.get_logger().info("Landing command detected (left key press).")
            time.sleep(0.1)
            self.drone(Landing())
            time.sleep(0.5)

        elif key == Key.right:
            self.get_logger().info("Takeoff command detected (right key press).")
            time.sleep(0.1)
            self.drone(TakeOff())
            time.sleep(0.5)

        elif hasattr(key, 'char') and key.char:
            if key.char == 'w':
                self.x_manual = 20
            elif key.char == 's':
                self.x_manual = -20
            elif key.char == 'a':
                self.y_manual = 20
            elif key.char == 'd':
                self.y_manual = -20
            elif key.char == 'r':
                self.z_manual = 10
            elif key.char == 'f':
                self.z_manual = -10            
            elif key.char == 'c':
                self.yaw_manual = 200
            elif key.char == 'x':
                self.yaw_manual = -200

            #self.get_logger().info(f"Manual control: x={self.x_manual}, y={self.y_manual}, z={self.z_manual}, yaw={self.yaw_manual}")

    def on_release(self, key):
        if hasattr(key, 'char') and key.char in ['w', 's']:
            self.x_manual = 0
        if hasattr(key, 'char') and key.char in ['a', 'd']:
            self.y_manual = 0
        if hasattr(key, 'char') and key.char in ['r', 'f']:
            self.z_manual = 0
        if hasattr(key, 'char') and key.char in ['x', 'c']:
            self.yaw_manual = 0

        #self.get_logger().info(f"Manual control released: x={self.x_manual}, y={self.y_manual}, z={self.z_manual}, yaw={self.yaw_manual}")


    def publish_pcmd(self):
 
        if self.drone == None:
            return

        if self.is_control_on is False and self.is_manual_on is True:
            #self.get_logger().info(f"Publishing manual PCMD: x={self.x_manual}, y={self.y_manual}, z={self.z_manual}, yaw={self.yaw_manual}")
            self.drone(PCMD(1,
                            -self.y_manual,
                            self.x_manual,
                            -self.yaw_manual,
                            self.z_manual,
                            timestampAndSeqNum=0,))
            
            self.control_output.control_x = self.x_manual
            self.control_output.control_y = self.y_manual
            self.control_output.control_z = self.z_manual
            self.control_output.control_yaw = self.yaw_manual
            
        elif self.is_control_on is True and self.is_manual_on is False:
            #self.get_logger().info(f"Publishing MPC PCMD: x={self.x_control}, y={self.y_control}, z={self.z_control}, yaw={self.yaw_control}")
            self.drone(PCMD(1,
                            -self.y_control,
                            self.x_control,
                            -self.yaw_control,
                            self.z_control,
                            timestampAndSeqNum=0,))
            
            self.control_output.control_x = self.x_control
            self.control_output.control_y = self.y_control
            self.control_output.control_z = self.z_control
            self.control_output.control_yaw = self.yaw_control
        
        else:
            self.get_logger().warn("Failed to publish PCMD from Sphinx.")

        self.pcmd_publisher.publish(self.control_output)

        

        



    def drone_tf_callback(self, msg):    
        for transform in msg.transforms:
            if transform.child_frame_id == self.target_frame:
                self.process_transform(transform)
    
    def process_transform(self, transform: TransformStamped):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        x = translation.x
        y = translation.y
        z = translation.z

        quaternion = (
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        )

        euler = transforms3d.euler.quat2euler(quaternion)
        yaw, pitch, roll = euler[0], euler[1], euler[2]
        
        self.drone_state.position.x = x
        self.drone_state.position.y = y
        self.drone_state.position.z = z 

        # if yaw >=0:
        #     self.drone_state.position.yaw = yaw - math.pi
        # elif yaw < 0:
        #     self.drone_state.position.yaw = yaw + math.pi

        self.drone_state.position.yaw = yaw

        self.drone_state.position.pitch = pitch + 0.0738

        if roll >=0:
            roll = -roll + math.pi 
        elif roll < 0:
            roll = -roll - math.pi
        self.drone_state.position.roll = roll - 0.0064



    def calculate_speed_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.previous_time is not None:
            if current_time - self.previous_time >= 0.02:
                speeds = {"x":0.0, "y":0.0, "z":0.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0}
                positions = {"x":self.drone_state.position.x, "y":self.drone_state.position.y, "z":self.drone_state.position.z,
                            "yaw": self.drone_state.position.yaw, "roll":self.drone_state.position.roll, "pitch":self.drone_state.position.pitch}
                previous_positions = {"x": self.previous_x, "y": self.previous_y, "z": self.previous_z,
                                    "yaw": self.previous_yaw, "roll":self.previous_roll, "pitch":self.previous_pitch}
                
                for index, axis in enumerate(speeds.keys()):
                    current_position = positions[axis]
                    previous_position = previous_positions[axis]
                    
                    if previous_position is not None:
                        if index >= 3:
                            if current_position < 0 and previous_position > 0 and current_position < -0.9 * math.pi and previous_position > 0.9 * math.pi:
                                delta_position = 2 * math.pi + current_position - previous_position
                            elif current_position > 0 and previous_position < 0 and current_position > 0.9 * math.pi and previous_position < -0.9 * math.pi:
                                delta_position = -2 * math.pi + current_position - previous_position
                            else:
                                delta_position = current_position - previous_position
                        else:
                            delta_position = current_position - previous_position

                        delta_time = current_time - self.previous_time
                        speeds[axis] = delta_position / delta_time
                self.previous_time = current_time
        
                self.drone_state.speed.x_speed_world = speeds["x"]
                self.drone_state.speed.y_speed_world = speeds["y"]
                self.drone_state.speed.z_speed = speeds["z"]
                self.drone_state.speed.yaw_speed = speeds["yaw"]
                self.drone_state.speed.roll_speed = speeds["roll"]
                self.drone_state.speed.pitch_speed = speeds["pitch"]

                self.previous_x = self.drone_state.position.x
                self.previous_y = self.drone_state.position.y
                self.previous_z = self.drone_state.position.z
                self.previous_yaw = self.drone_state.position.yaw
                self.previous_roll = self.drone_state.position.roll
                self.previous_pitch = self.drone_state.position.pitch

        if self.previous_time_update == True:
            self.previous_time = current_time
            self.previous_time_update = False


    def start_user_input_thread(self):
        input_thread = threading.Thread(target=self.handle_user_input)
        input_thread.daemon = True
        input_thread.start()

    def handle_user_input(self):
        while rclpy.ok():
            try:
                user_input = input('Enter [0:calibrate or 1:x or 2:y or 3:z or 4:yaw]: ')
                if user_input in ['0', '1', '2', '3', '4']:
                    self.test_axis = user_input
                    print(f"Test axis set to {self.test_axis}. Starting the test...")

                    start_command = input('Type "5" to begin the test: ')
                    if start_command == "5":                    
                        self.is_test_on = True
                        self.is_control_on = True
                        self.is_manual_on = False
                        self.csv_file = os.path.join(self.data_dir, f'state_data_{self.test_axis}.csv')
                        self.write_csv_header()  # Write CSV header after setting the file name
                        self.process()
                        break
                    else: 
                        print("Invalid command. Please type '5' to begin.")
                else:
                    print("Invalid input. Please enter 1, 2, 3, or 4.")
            except ValueError:
                print("Invalid input!")

    def write_csv_header(self):
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['elapsed_time',
                             'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                             'x_speed', 'y_speed', 'z_speed', 'roll_speed', 'pitch_speed', 'yaw_speed',
                             'control_x', 'control_y', 'control_z', 'control_yaw'])



    def process(self):

        if self.running:
            try:
                time.sleep(5)  # Additional delay if needed
                self.write_data_flag = True 

                for _ in range(1):
                    if self.test_axis == '0':
                        self.collect_and_publish_pcmd(duration=5.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                    if self.test_axis == '1':
                        self.collect_and_publish_pcmd(duration=1.0, x_input=40, y_input=0, z_input=0, yaw_input=0)
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                    elif self.test_axis == '2':
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=40, z_input=0, yaw_input=0)
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                    elif self.test_axis == '3':
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=40, yaw_input=0)
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                    elif self.test_axis == '4':
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=100)
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                self.write_data_flag = False  # Stop writing data after test
                self.is_control_on = False
                self.is_manual_on = True

                self.x_control = 0
                self.y_control = 0
                self.z_control = 0
                self.yaw_control = 0
                
            finally:
                print('Test finished')




    def collect_and_publish_pcmd(self, duration, x_input, y_input, z_input, yaw_input):

        self.control_output.control_x = int(x_input)
        self.control_output.control_y = int(y_input)
        self.control_output.control_z = int(z_input)
        self.control_output.control_yaw = int(yaw_input)

        self.x_control = int(x_input)
        self.y_control = int(y_input)
        self.z_control = int(z_input)
        self.yaw_control = int(yaw_input)

        time.sleep(duration)


    def save_data_thread_callback(self):
        while self.running:
            next_time = time.perf_counter()
            if self.write_data_flag:
                # self.get_logger().info("Saving data...")
                formatted_time_stamp = f"{self.time_stamp:.2f}"
                data = [formatted_time_stamp,
                        self.drone_state.position.x, 
                        self.drone_state.position.y,
                        self.drone_state.position.z,
                        self.drone_state.position.roll,
                        self.drone_state.position.pitch,
                        self.drone_state.position.yaw,
                        self.drone_state.speed.x_speed_world,
                        self.drone_state.speed.y_speed_world,
                        self.drone_state.speed.z_speed,
                        self.drone_state.speed.roll_speed,
                        self.drone_state.speed.pitch_speed,
                        self.drone_state.speed.yaw_speed,
                        self.control_output.control_x, 
                        self.control_output.control_y, 
                        self.control_output.control_z, 
                        self.control_output.control_yaw]
                with open(self.csv_file, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(data)
                self.time_stamp += 1 / self.freq

                self.publisher.publish(self.collect_current_state)

            next_time += 1 / self.freq
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)



    def Connect(self):
        self.get_logger().info('Connecting to Anafi drone...')
        self.DRONE_IP = os.getenv("DRONE_IP", "192.168.42.1")
        #self.DRONE_IP = os.getenv("DRONE_IP", "10.202.0.1")
        self.drone = olympe.Drone(self.DRONE_IP)

        for i in range(5):
            if self.running:
                connection = self.drone.connect(retry=1)
                if connection:
                    self.connected = True
                    self.get_logger().info('Connected to Anafi drone!')
                    break
                else:
                    self.get_logger().info(f'Trying to connect (%d)' % (i + 1))
                    time.sleep(2)

        if not self.connected:
            self.get_logger().info("Failed to connect.")

    def Stop(self):
        self.running = False

def main(args=None):
    rclpy.init(args=args)    
    node = CollectDataNode()
    node.Connect()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
