import numpy as np
import pandas as pd
from xarm.wrapper import XArmAPI
import time
from scipy.signal import butter, filtfilt

class IMUProcessor:
    def __init__(self, filepath):
        self.filepath = filepath
        self.sampling_rate = 100  # Hz from IMU data
        self.scale_factor = 300  # Scale factor for robot workspace
        self.max_reach = 700  # Maximum reach of xArm in mm
        
    def butter_lowpass_filter(self, data, cutoff=2.0, order=4):
        nyq = 0.5 * self.sampling_rate
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return filtfilt(b, a, data)
        
    def process_data(self):
        # Read IMU data
        print("Reading IMU data...")
        data = pd.read_csv(self.filepath, delimiter='\t')
        
        # Extract acceleration and orientation
        acc_x = data['AccX(g)'].values
        acc_y = data['AccY(g)'].values
        acc_z = data['AccZ(g)'].values - 1.0  # Remove gravity
        
        # Get angles
        angle_x = data['AngleX(°)'].values
        angle_y = data['AngleY(°)'].values
        angle_z = data['AngleZ(°)'].values
        
        # Filter acceleration data
        print("Filtering acceleration data...")
        acc_x_filtered = self.butter_lowpass_filter(acc_x)
        acc_y_filtered = self.butter_lowpass_filter(acc_y)
        acc_z_filtered = self.butter_lowpass_filter(acc_z)
        
        # Convert to m/s^2
        acc_x_filtered *= 9.81
        acc_y_filtered *= 9.81
        acc_z_filtered *= 9.81
        
        # Integrate to get velocity and position
        dt = 1.0 / self.sampling_rate
        vel_x = np.cumsum(acc_x_filtered) * dt
        vel_y = np.cumsum(acc_y_filtered) * dt
        vel_z = np.cumsum(acc_z_filtered) * dt
        
        pos_x = np.cumsum(vel_x) * dt * self.scale_factor
        pos_y = np.cumsum(vel_y) * dt * self.scale_factor
        pos_z = np.cumsum(vel_z) * dt * self.scale_factor
        
        # Clip to robot workspace
        pos_x = np.clip(pos_x, -self.max_reach, self.max_reach)
        pos_y = np.clip(pos_y, -self.max_reach, self.max_reach)
        pos_z = np.clip(pos_z, -400, 951.5)  # xArm6 Z range
        
        # Create trajectory points
        trajectory = []
        sample_interval = max(1, len(pos_x) // 50)  # Sample ~50 points
        
        print("\nTrajectory Points (X, Y, Z in mm, Roll, Pitch, Yaw in degrees):")
        print("Point | X (mm)  | Y (mm)  | Z (mm)  | Roll°  | Pitch° | Yaw°")
        print("-" * 70)
        
        point_count = 0
        for i in range(0, len(pos_x), sample_interval):
            point = [
                float(pos_x[i]),
                float(pos_y[i]),
                float(pos_z[i]),
                float(angle_x[i]),
                float(angle_y[i]),
                float(angle_z[i])
            ]
            trajectory.append(point)
            
            # Print all points with fixed width formatting
            print(f"{point_count:3d}   | {point[0]:7.1f} | {point[1]:7.1f} | {point[2]:7.1f} | {point[3]:6.1f} | {point[4]:6.1f} | {point[5]:6.1f}")
            point_count += 1
        
        print("\nWorkspace Analysis:")
        print(f"Total trajectory points: {len(trajectory)}")
        print("\nWorkspace bounds:")
        print(f"X range: {np.min(pos_x):.1f} to {np.max(pos_x):.1f} mm (limit: ±{self.max_reach} mm)")
        print(f"Y range: {np.min(pos_y):.1f} to {np.max(pos_y):.1f} mm (limit: ±{self.max_reach} mm)")
        print(f"Z range: {np.min(pos_z):.1f} to {np.max(pos_z):.1f} mm (limit: -400 to 951.5 mm)")
        
        # Print angle ranges
        print("\nOrientation ranges:")
        print(f"Roll : {np.min(angle_x):.1f}° to {np.max(angle_x):.1f}°")
        print(f"Pitch: {np.min(angle_y):.1f}° to {np.max(angle_y):.1f}°")
        print(f"Yaw  : {np.min(angle_z):.1f}° to {np.max(angle_z):.1f}°")
        
        return trajectory

def main():
    # Initialize IMU processor
    processor = IMUProcessor("C:\/Users\/panka\/Desktop\/PMC\/PMC\/20250127100932.txt")
    
    # Process IMU data and get trajectory
    trajectory = processor.process_data()
    
    # Ask user if they want to proceed with robot motion
    proceed = input("\nWould you like to proceed with robot motion? (yes/no): ")
    
    if proceed.lower() != 'yes':
        print("Motion execution cancelled")
        return
        
    print("\nInitializing robot motion...")
    
    # Robot motion code - commented out for safety until coordinates are verified
    """
    # Initialize robot
    arm = XArmAPI('192.168.1.211')
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    # Set motion parameters
    arm.set_tcp_jerk(2000)
    arm.set_tcp_maxacc(1000)
    
    # Execute trajectory
    for i, point in enumerate(trajectory):
        print(f"Moving to point {i+1}/{len(trajectory)}")
        arm.set_position(x=point[0], y=point[1], z=point[2],
                        roll=point[3], pitch=point[4], yaw=point[5],
                        speed=100, wait=True)
    
    # Return to home position
    print("Returning to home position...")
    arm.move_gohome()
    """

if __name__ == "__main__":
    main()