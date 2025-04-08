import numpy as np
import csv
import os
from scipy.optimize import minimize

class CalculateStateFunction:
    def __init__(self):
        # Initialize A and B matrices
        self.A = np.eye(8)
        self.A[0, 4] = 0.04
        self.A[1, 5] = 0.04
        self.A[2, 6] = 0.04
        self.A[3, 7] = 0.04
        self.B = np.zeros((8, 4))
        
        data_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data', 'anafi_state_function', 'linear_mpc')
        input_csv_files = [
            os.path.join(data_dir, 'state_data_x.csv'),
            os.path.join(data_dir, 'state_data_y.csv'),
            os.path.join(data_dir, 'state_data_z.csv'),
            os.path.join(data_dir, 'state_data_yaw.csv'),
        ]
        axis = ['x', 'y', 'z', 'yaw']

        self.csv_dir = os.path.join(data_dir, 'state_function_matrix')
        os.makedirs(self.csv_dir, exist_ok=True)

        self.positions_x, self.controls_x = self.load_and_extract_data(input_csv_files[0], axis[0])
        self.positions_y, self.controls_y = self.load_and_extract_data(input_csv_files[1], axis[1])
        self.positions_z, self.controls_z = self.load_and_extract_data(input_csv_files[2], axis[2])
        self.positions_yaw, self.controls_yaw = self.load_and_extract_data(input_csv_files[3], axis[3])
        
        self.train_models()

    def load_and_extract_data(self, filename, axis):
        with open(filename, 'r') as file:
            positions = []
            controls = []
            reader = csv.reader(file)
            headers = next(reader)
            pos_col = headers.index(axis)
            control_col = headers.index(f'control_{axis}')
            for row in reader:
                positions.append(float(row[pos_col]))
                controls.append(float(row[control_col]))

        return np.array(positions).reshape(-1,1), np.array(controls).reshape(-1, 1)
    
    def train_models(self):

        def error_function(params, x_real, u_real, delta_t):
            A, B = params
            error = 0.0
            x_estimated = x_real[0]
            v_k_current = np.zeros(len(x_real))

            for k in range(len(v_k_current) - 1):
                v_k_plus_1 = A * v_k_current[k] + B * u_real[k]
                x_estimated = x_estimated + delta_t * v_k_current[k]
                error += (x_real[k + 1] - x_estimated) ** 2
                v_k_current[k + 1] = v_k_plus_1[0]

            return error

        delta_t = 0.04
        initial_guess = np.array([1.0, 0.0])

        print("Training model for x...")
        self.result_x = minimize(error_function, initial_guess, args=(self.positions_x, self.controls_x, delta_t))
        print("Training model for y...")
        self.result_y = minimize(error_function, initial_guess, args=(self.positions_y, self.controls_y, delta_t))
        print("Training model for z...")
        self.result_z = minimize(error_function, initial_guess, args=(self.positions_z, self.controls_z, delta_t))
        print("Training model for yaw...")
        self.result_yaw = minimize(error_function, initial_guess, args=(self.positions_yaw, self.controls_yaw, delta_t))

        self.save_matrices()

    def save_matrices(self):
        def save_matrix(result, idx):
            A_opt, B_opt = result.x
            self.A[4 + idx, 4 + idx] = A_opt
            self.B[4 + idx, idx] = B_opt

            if idx == 3:
                np.savetxt(os.path.join(self.csv_dir, 'A_matrix.csv'), self.A, delimiter=',', fmt='%.5f')
                np.savetxt(os.path.join(self.csv_dir, 'B_matrix.csv'), self.B, delimiter=',', fmt='%.5f')

        save_matrix(self.result_x, 0)
        save_matrix(self.result_y, 1)
        save_matrix(self.result_z, 2)
        save_matrix(self.result_yaw, 3)

def main():
    CalculateStateFunction()

if __name__ == '__main__':
    main()
