import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from dtaidistance import dtw

def load_motion_data(filepath, skip_metadata=13):
    """Load and clean the 6D motion data file."""
    df = pd.read_csv(filepath, sep='\t', skiprows=skip_metadata)
    df = df.drop(columns=[col for col in df.columns if 'Unnamed' in col], errors='ignore')
    return df

def compute_motion_features(df):
    """Compute linear and rotational differences, magnitudes, and accelerations."""
    # First derivatives (velocities)
    df_copy = df[['cube X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']]
    
    df_copy['dx'] = df_copy['cube X'].diff()
    df_copy['dy'] = df_copy['Y'].diff()
    df_copy['dz'] = df_copy['Z'].diff()
    df_copy['droll'] = df_copy['Roll'].diff()
    df_copy['dpitch'] = df_copy['Pitch'].diff()
    df_copy['dyaw'] = df_copy['Yaw'].diff()
    
    # Linear and rotational movement magnitudes
    df_copy['linear_movement'] = np.sqrt(df_copy['dx']**2 + df_copy['dy']**2 + df_copy['dz']**2)
    df_copy['rotational_movement'] = np.sqrt(df_copy['droll']**2 + df_copy['dpitch']**2 + df_copy['dyaw']**2)
    
    # Second derivatives (accelerations)
    df_copy['ddx'] = df_copy['dx'].diff()  # Linear acceleration in X
    df_copy['ddy'] = df_copy['dy'].diff()  # Linear acceleration in Y
    df_copy['ddz'] = df_copy['dz'].diff()  # Linear acceleration in Z
    df_copy['ddroll'] = df_copy['droll'].diff()  # Rotational acceleration in Roll
    df_copy['ddpitch'] = df_copy['dpitch'].diff()  # Rotational acceleration in Pitch
    df_copy['ddyaw'] = df_copy['dyaw'].diff()  # Rotational acceleration in Yaw
    
    # Linear and rotational acceleration magnitudes
    df_copy['linear_acceleration'] = np.sqrt(df_copy['ddx']**2 + df_copy['ddy']**2 + df_copy['ddz']**2)
    df_copy['rotational_acceleration'] = np.sqrt(df_copy['ddroll']**2 + df_copy['ddpitch']**2 + df_copy['ddyaw']**2)
    
    return df_copy
    

def classify_motion_type(df, linear_thresh=0.5, rotational_thresh=0.5):
    """Classify motion type based on movement magnitude."""
    def classify(row):
        if row['linear_movement'] < linear_thresh and row['rotational_movement'] < rotational_thresh:
            return 'Stationary'
        elif row['linear_movement'] >= row['rotational_movement']:
            return 'Linear'
        else:
            return 'Rotational'
    df['motion_type'] = df.apply(classify, axis=1)
    return df

def summarize_motion(df):
    """Summarize motion counts, dominant movement axis, and acceleration statistics."""
    motion_counts = df['motion_type'].value_counts()
    
    # Movement axis totals
    axis_totals = {
        'X-axis': df['dx'].abs().sum(),
        'Y-axis': df['dy'].abs().sum(),
        'Z-axis': df['dz'].abs().sum()
    }
    dominant_axis = max(axis_totals, key=axis_totals.get)
    
    # Acceleration axis totals
    acceleration_totals = {
        'X-axis': df['ddx'].abs().sum(),
        'Y-axis': df['ddy'].abs().sum(),
        'Z-axis': df['ddz'].abs().sum()
    }
    dominant_accel_axis = max(acceleration_totals, key=acceleration_totals.get)
    
    # Acceleration statistics
    acceleration_stats = {
        'max_linear_acceleration': df['linear_acceleration'].max(),
        'mean_linear_acceleration': df['linear_acceleration'].mean(),
        'max_rotational_acceleration': df['rotational_acceleration'].max(),
        'mean_rotational_acceleration': df['rotational_acceleration'].mean(),
        'linear_acceleration_std': df['linear_acceleration'].std(),
        'rotational_acceleration_std': df['rotational_acceleration'].std()
    }
    
    return motion_counts, axis_totals, dominant_axis, acceleration_totals, dominant_accel_axis, acceleration_stats

def plot_motion_data(df, save_path=None):
    """Plot positions, rotations, magnitudes, accelerations, and motion types."""
    fig, axs = plt.subplots(4, 2, figsize=(15, 12))
    fig.suptitle('Movement Analysis Over Time', fontsize=16)

    # Position plots
    axs[0, 0].plot(df['Time'], df['cube X'], label='X')
    axs[0, 0].plot(df['Time'], df['Y'], label='Y')
    axs[0, 0].plot(df['Time'], df['Z'], label='Z')
    axs[0, 0].set_title('Position (mm)')
    axs[0, 0].legend()

    # Linear movement magnitude
    axs[0, 1].plot(df['Time'], df['linear_movement'], color='blue')
    axs[0, 1].set_title('Linear Movement per Frame')

    # Rotation plots
    axs[1, 0].plot(df['Time'], df['Roll'], label='Roll')
    axs[1, 0].plot(df['Time'], df['Pitch'], label='Pitch')
    axs[1, 0].plot(df['Time'], df['Yaw'], label='Yaw')
    axs[1, 0].set_title('Rotation (degrees)')
    axs[1, 0].legend()

    # Rotational movement magnitude
    axs[1, 1].plot(df['Time'], df['rotational_movement'], color='green')
    axs[1, 1].set_title('Rotational Movement per Frame')

    # Linear acceleration plots
    axs[2, 0].plot(df['Time'], df['ddx'], label='X accel', alpha=0.7)
    axs[2, 0].plot(df['Time'], df['ddy'], label='Y accel', alpha=0.7)
    axs[2, 0].plot(df['Time'], df['ddz'], label='Z accel', alpha=0.7)
    axs[2, 0].set_title('Linear Acceleration (mm/frame²)')
    axs[2, 0].legend()

    # Linear acceleration magnitude
    axs[2, 1].plot(df['Time'], df['linear_acceleration'], color='red')
    axs[2, 1].set_title('Linear Acceleration Magnitude')

    # Rotational acceleration plots
    axs[3, 0].plot(df['Time'], df['ddroll'], label='Roll accel', alpha=0.7)
    axs[3, 0].plot(df['Time'], df['ddpitch'], label='Pitch accel', alpha=0.7)
    axs[3, 0].plot(df['Time'], df['ddyaw'], label='Yaw accel', alpha=0.7)
    axs[3, 0].set_title('Rotational Acceleration (degrees/frame²)')
    axs[3, 0].legend()

    # Motion type over time
    axs[3, 1].plot(df['Time'], df['motion_type'].map({'Stationary': 0, 'Linear': 1, 'Rotational': 2}),
                  drawstyle='steps-post', color='purple')
    axs[3, 1].set_title('Motion Type Over Time')
    axs[3, 1].set_yticks([0, 1, 2])
    axs[3, 1].set_yticklabels(['Stationary', 'Linear', 'Rotational'])
    axs[3, 1].set_xlabel('Time (s)')

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"Plot saved to {save_path}")
    else:
        print("Plot not saved, display only.")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

def compare_motion_files(file1, file2, save_plot=False):
    df1 = compute_motion_features(load_motion_data(file1))
    df2 = compute_motion_features(load_motion_data(file2))

    min_len = min(len(df1), len(df2))
    df1 = df1.iloc[:min_len]
    df2 = df2.iloc[:min_len]

    distances = {}
    for axis in ['cube X', 'Y', 'Z']:
        series1 = df1[axis].values
        series2 = df2[axis].values
        distances[axis] = dtw.distance(series1, series2)

    print("DTW Distances between motion files:")
    for axis, dist in distances.items():
        print(f"  {axis}: {dist:.2f}")

    plt.figure(figsize=(10, 5))
    plt.plot(df1['Time'], df1['cube X'], label='File 1 - X', alpha=0.7)
    plt.plot(df2['Time'], df2['cube X'], label='File 2 - X', alpha=0.7)
    plt.title('Comparison of Cube X Movement')
    plt.xlabel('Time (s)')
    plt.ylabel('Cube X Position (mm)')
    plt.legend()

    if save_plot:
        plot_save_path = f'{os.path.dirname(file1)}/{os.path.basename(file1).split(".")[0]}_vs_{os.path.basename(file2).split(".")[0]}_comparison.png'
        os.makedirs(os.path.dirname(plot_save_path), exist_ok=True)
        
        plt.savefig(plot_save_path, dpi=300)
        print(f"Comparison plot saved to {plot_save_path}")
    else:
        plt.show()


def identify_motion_type(tsv_path):
    # rootdir = os.path.expanduser('~/Downloads/cube-all')  # Now ~ gets expanded to your home directory
    # filepath = f'{rootdir}/cube-stationary_6D.tsv'  # Update this path if needed
    print("Loading motion data from:", tsv_path)
    df = load_motion_data(tsv_path)
    df = compute_motion_features(df)
    df = classify_motion_type(df)
    motion_counts, axis_totals, dominant_axis, acceleration_totals, dominant_accel_axis, acceleration_stats = summarize_motion(df)

    print("Motion Type Counts:\n", motion_counts)
    print("Total Movement per Axis:\n", axis_totals)
    print("Dominant Axis of Movement:", dominant_axis)
    print("Acceleration Statistics:\n", acceleration_stats)
    print("Total Acceleration per Axis:\n", acceleration_totals)
    print("Dominant Axis of Acceleration:", dominant_accel_axis)

    save_path = f'{os.path.dirname(tsv_path)}/{tsv_path.split("/")[-1].split(".")[0]}_summary.txt'
    # check if parent directory exists, if not create it
    print(save_path)
    
    with open(save_path, 'w') as f:
        f.write("Motion Type Counts:\n")
        f.write(motion_counts.to_string())
        f.write("\n\nTotal Movement per Axis:\n")
        f.write(pd.Series(axis_totals).to_string())
        f.write("\n\nDominant Axis of Movement:\n")
        f.write(dominant_axis)
        f.write("\n\nAcceleration Statistics:\n")
        f.write(pd.Series(acceleration_stats).to_string())
        f.write("\n\nTotal Acceleration per Axis:\n")
        f.write(pd.Series(acceleration_totals).to_string())
        f.write("\n\nDominant Axis of Acceleration:\n")
        f.write(dominant_accel_axis)

    plot_save_path = f'{os.path.dirname(tsv_path)}/{tsv_path.split("/")[-1].split(".")[0]}_motion_analysis_plot.png'
    # Ensure the directory exists for the plot
    os.makedirs(os.path.dirname(plot_save_path), exist_ok=True)
    
    plot_motion_data(df, save_path=plot_save_path)

if __name__ == '__main__':
    # main()
    rootdir = os.path.expanduser('~/Downloads/cube-all')  # Now ~ gets expanded to your home directory
    # filepath = f'{rootdir}/cube-stationary_6D.tsv'  # Update this path if needed
    
    # identify_motion_type(filepath)
    file1 = f'{rootdir}/cube-y_6D.tsv'
    file2 = f'{rootdir}/cube-x_6D.tsv'
    compare_motion_files(file1, file2, save_plot=True)
