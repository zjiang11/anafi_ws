import numpy as np
import csv
import os
import matplotlib.pyplot as plt
import math

class TestStateFunction:
    def __init__(self):
        self.state = np.zeros(8)
        self.previous_state = np.zeros(8)
        self.input = np.zeros(4)
        self.timestamp = 0.0
        self.freq = 25
        self.time_step = 1 / self.freq
        self.simulation_duration = 10  # seconds

        data_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data', 'anafi_state_function', 'linear_mpc', 'state_function_matrix')

        self.A = np.loadtxt(os.path.join(data_dir, 'A_matrix.csv'), delimiter=',')
        self.B = np.loadtxt(os.path.join(data_dir, 'B_matrix.csv'), delimiter=',')

        self.simulated_states = []
        self.real_states = {
            'x': [],
            'y': [],
            'z': [],
            'yaw': []
        }

        self.time_axis = []

        file_names = [
            ('state_data_x.csv', 'x'),
            ('state_data_y.csv', 'y'),
            ('state_data_z.csv', 'z'),
            ('state_data_yaw.csv', 'yaw'),
        ]

        for file_name, state_key in file_names:
            self.read_real_data(file_name, state_key)

        self.run_simulation()
        self.plot_states()

    def read_real_data(self, file_name, state_key):
        data_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data', 'anafi_state_function', 'linear_mpc')
        csv_file = os.path.join(data_dir, file_name)

        with open(csv_file, mode='r') as file:
            reader = csv.DictReader(file)
            initial_position = None
            for row in reader:
                timestamp = float(row['elapsed_time'])
                if timestamp <= self.simulation_duration:
                    yaw = float(row['yaw'])
                    if yaw < 0 and initial_position is not None:
                        yaw += 2 * math.pi

                    current_position = [
                        float(row['x']), float(row['y']), float(row['z']), yaw,
                        float(row['x_speed']), float(row['y_speed']), float(row['z_speed']), float(row['yaw_speed'])
                    ]
                    if initial_position is None:
                        initial_position = current_position[:4]
                    normalized_position = [
                        current_position[0] - initial_position[0],
                        current_position[1] - initial_position[1],
                        current_position[2] - initial_position[2],
                        current_position[3] - initial_position[3],
                        current_position[4],
                        current_position[5],
                        current_position[6],
                        current_position[7]
                    ]
                    self.real_states[state_key].append(normalized_position)
                    self.time_axis.append(timestamp)

    def run_simulation(self):
        input_sequences = {
            'x': [20, 0, 0, 0] * (self.freq) + [0, 0, 0, 0] * (self.freq * 10),
            'y': [0, 20, 0, 0] * (int(0.5 * self.freq) + 1) + [0, 0, 0, 0] * (self.freq * 10),
            'z': [0, 0, 20, 0] * self.freq + [0, 0, 0, 0] * self.freq + [0, 0, -20, 0] * self.freq + [0, 0, 0, 0] * self.freq,
            'yaw': [0, 0, 0, 100] * self.freq + [0, 0, 0, 0] * self.freq + [0, 0, 0, -100] * self.freq + [0, 0, 0, 0] * self.freq
        }

        for key, input_sequence in input_sequences.items():
            simulated_states = self.simulate(input_sequence)
            self.simulated_states.append(simulated_states)

    def simulate(self, input_sequence):
        states = []
        self.previous_state = np.zeros(8)
        for i in range(len(input_sequence) // 4):
            self.input = input_sequence[i * 4:(i + 1) * 4]
            self.state = self.A.dot(self.previous_state) + self.B.dot(self.input)
            states.append(self.state.copy())
            self.previous_state = self.state
        return np.array(states)

    def plot_states(self):
        savedir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data', 'anafi_state_function', 'linear_mpc', 'plot_real_simulated_comparison')
        os.makedirs(savedir, exist_ok=True)
        labels = ['x', 'y', 'z', 'yaw', 'x_speed', 'y_speed', 'z_speed', 'yaw_speed']

        for i, key in enumerate(self.real_states.keys()):
            real_states = np.array(self.real_states[key])
            if i == 3:  # Adjust yaw
                real_states[real_states > np.pi] -= 2 * np.pi

            simulated_states = self.simulated_states[i]
            time_axis = self.time_axis[:len(real_states)]
            min_length = min(len(simulated_states), len(real_states), len(time_axis))

            # Position plot
            fig_pos = plt.figure(figsize=(10, 6))
            plt.plot(time_axis[:min_length], simulated_states[:min_length, i], label='Estimated by System State')
            if min_length > 0:
                plt.plot(time_axis[:min_length], real_states[:min_length, i], label='Measured by Vicon System', linestyle='dashed')
            plt.xlabel("Time (s)", fontsize=12)
            plt.ylabel(f"{key.upper()} Position (m)" if i != 3 else "Angle (rad)", fontsize=12)
            plt.legend(loc='upper right', fontsize=12)
            if i == 0:
                plt.ylim([0.0, 4.5])
            elif i == 1:
                plt.ylim([0.0, 3.5])
            save_path_pos = os.path.join(savedir, f"position_comparison_{labels[i]}.png")
            plt.savefig(save_path_pos, dpi=300, bbox_inches="tight")
            plt.show()

            # Speed plot
            fig_speed = plt.figure(figsize=(10, 6))
            plt.plot(time_axis[:min_length], simulated_states[:min_length, i + 4], label='Estimated by System State')
            if min_length > 0:
                plt.plot(time_axis[:min_length], real_states[:min_length, i + 4], label='Measured by Vicon System', linestyle='dashed')
            plt.xlabel("Time (s)", fontsize=12)
            plt.ylabel(f"{key.upper()} Speed (m/s)" if i != 3 else "Angular Speed (rad/s)", fontsize=12)
            plt.legend(loc='upper right', fontsize=12)
            save_path_speed = os.path.join(savedir, f"speed_comparison_{labels[i]}.png")
            plt.savefig(save_path_speed, dpi=300, bbox_inches="tight")
            plt.show()


def main():
    TestStateFunction()

if __name__ == '__main__':
    main()
