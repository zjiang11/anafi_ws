import pandas as pd
import os


data_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data', 'anafi_state_function', 'newton_euler_mpc')

read_file_roll = os.path.join(data_dir, "state_data_calibrate.csv")
save_file = os.path.join(data_dir, "vicon_calibrate_roll_pitch.csv")
df = pd.read_csv(read_file_roll)

roll_average = df[['roll']].mean().values[0]
pitch_average = df[['pitch']].mean().values[0]

averages_df = pd.DataFrame({'roll_average': [roll_average], 'pitch_average': [pitch_average]})
averages_df.to_csv(save_file, index=False)