# To use the code:

# Update the ARM_IP with your robot's IP address
# Set the correct path to your IMU data file
# Run the script
# Review the trajectory visualization
# Press Enter to start robot movement

# The script will:

# Process the IMU data
# Show you the planned trajectory
# Wait for your confirmation
# Execute the movement
# Return to home position

# You can adjust the scale_factor in the process_data method to change the magnitude of the motion.





import numpy as np
from xarm.wrapper import XArmAPI
import pandas as pd
import time
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class XArm6Workspace:
    """Handles workspace constraints for xArm 6"""
    def __init__(self):
        # Robot physical constraints
        self.MAX_REACH = 700.0  # mm
        self.MIN_REACH = 0.0    # mm
        self.Z_MIN = -400.0     # mm
        self.Z_MAX = 951.5      # mm
        
        # Joint limits in degrees
        self.JOINT_LIMITS = {
            'J2': (-118.0, 120.0),
            'J3': (-225.0, 11.0),
            'J5': (-97.0, 180.0)
        }

    def constrain_to_workspace(self, position):
        """Constrain position to valid workspace"""
        x, y, z = position
        
        # Calculate radius from base
        r = np.sqrt(x*x + y*y)
        
        # Constrain radius to maximum reach
        if r > self.MAX_REACH:
            scale = self.MAX_REACH / r
            x *= scale
            y *= scale
        
        # Constrain Z to valid range
        z = np.clip(z, self.Z_MIN, self.Z_MAX)
        
        return np.array([x, y, z])

class IMUDataProcessor:
    """Processes IMU data from file and generates robot trajectory"""
    def __init__(self, filepath):
        self.filepath = filepath
        self.workspace = XArm6Workspace()
        self.sampling_rate = 100  # Hz (from IMU data)
        
    def read_imu_data(self):
        """Read and parse IMU data from file"""
        try:
            # Skip the first row (header) and read data
            data = pd.read_csv(self.filepath, delimiter='\t', skiprows=1)
            return data
        except Exception as e:
            print(f"Error reading file: {e}")
            return None

    def butter_lowpass_filter(self, data, cutoff, fs, order=4):
        """Apply Butterworth low-pass filter"""
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return filtfilt(b, a, data)

    def process_data(self):
        """Process IMU data and generate trajectory"""
        data = self.read_imu_data()
        if data is None:
            return None

        # Extract acceleration data
        acc_x = data['AccX(g)'].values
        acc_y = data['AccY(g)'].values
        acc_z = data['AccZ(g)'].values - 1.0  # Remove gravity

        # Filter acceleration data
        cutoff = 2.0  # Hz
        acc_x_filtered = self.butter_lowpass_filter(acc_x, cutoff, self.sampling_rate)
        acc_y_filtered = self.butter_lowpass_filter(acc_y, cutoff, self.sampling_rate)
        acc_z_filtered = self.butter_lowpass_filter(acc_z, cutoff, self.sampling_rate)

        # Calculate velocities and positions
        dt = 1.0 / self.sampling_rate
        vel_x = np.cumsum(acc_x_filtered) * dt * 9.81
        vel_y = np.cumsum(acc_y_filtered) * dt * 9.81
        vel_z = np.cumsum(acc_z_filtered) * dt * 9.81

        # Scale factor to map to robot workspace
        scale_factor = 300  # Adjust this to change motion magnitude
        pos_x = np.cumsum(vel_x) * dt * scale_factor
        pos_y = np.cumsum(vel_y) * dt * scale_factor
        pos_z = np.cumsum(vel_z) * dt * scale_factor

        # Create trajectory points
        trajectory = []
        for i in range(len(pos_x)):
            # Get position
            position = self.workspace.constrain_to_workspace([pos_x[i], pos_y[i], pos_z[i]])
            
            # Get orientation (angles)
            roll = data['AngleX(°)'].values[i]
            pitch = data['AngleY(°)'].values[i]
            yaw = data['AngleZ(°)'].values[i]
            
            # Combine position and orientation
            point = np.concatenate([position, [roll, pitch, yaw]])
            trajectory.append(point)

        return np.array(trajectory)

    def visualize_trajectory(self, trajectory):
        """Visualize the processed trajectory"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2])
        
        # Set labels
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('Robot Trajectory')
        
        # Set axis limits based on workspace
        ax.set_xlim([-self.workspace.MAX_REACH, self.workspace.MAX_REACH])
        ax.set_ylim([-self.workspace.MAX_REACH, self.workspace.MAX_REACH])
        ax.set_zlim([self.workspace.Z_MIN, self.workspace.Z_MAX])
        
        plt.show()

class XArm6Controller:
    """Controls xArm 6 robot movement"""
    def __init__(self, ip_address):
        self.arm = XArmAPI(ip_address)
        self.setup_robot()
        
    def setup_robot(self):
        """Initialize robot settings"""
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)  # Position control mode
        self.arm.set_state(0)  # Start state
        self.arm.set_tcp_load(1.0, [0, 0, 0])
        self.arm.set_tcp_maxacc(1000)
        self.arm.set_tcp_jerk(1000)
        
        # Move to home position
        self.move_home()
        
    def move_home(self):
        """Move robot to home position"""
        home_position = [200, 0, 200, 180, 0, 0]  # Adjust as needed
        self.arm.set_position(*home_position, wait=True)
        print("Robot at home position")

    def execute_trajectory(self, trajectory, speed=100):
        """Execute the given trajectory"""
        try:
            print("Starting trajectory execution...")
            
            for point in trajectory:
                # Check if position is reachable
                code = self.arm.is_tcp_limit(point[:3])
                if code != 0:
                    print(f"Position {point[:3]} is outside TCP limits, skipping...")
                    continue
                
                # Move to position
                code = self.arm.set_position(
                    x=point[0], y=point[1], z=point[2],
                    roll=point[3], pitch=point[4], yaw=point[5],
                    speed=speed,
                    wait=True,
                    is_radian=False
                )
                
                if code != 0:
                    print(f"Movement error, code: {code}")
                
                time.sleep(0.01)  # Small delay between movements
                
            print("Trajectory execution completed")
            
            # Return to home position
            self.move_home()
            
        except Exception as e:
            print(f"Error during trajectory execution: {e}")
            self.move_home()
        finally:
            print("Ensuring robot is in a safe state...")
            self.arm.set_state(4)  # Stop state

def main():
    # Configuration
    ARM_IP = "192.168.1.211"  # Update with your robot's IP
    IMU_DATA_FILE = "20250127100932.txt"  # Update with your file path
    
    try:
        # Initialize data processor
        processor = IMUDataProcessor(IMU_DATA_FILE)
        
        # Process IMU data
        trajectory = processor.process_data()
        if trajectory is None:
            print("Failed to process IMU data")
            return
        
        # Visualize trajectory
        processor.visualize_trajectory(trajectory)
        
        # Initialize robot controller
        controller = XArm6Controller(ARM_IP)
        
        # Execute trajectory
        input("Press Enter to start robot movement...")
        controller.execute_trajectory(trajectory)
        
    except Exception as e:
        print(f"Error in main execution: {e}")
    finally:
        if 'controller' in locals():
            controller.move_home()
            controller.arm.disconnect()

if __name__ == "__main__":
    main()