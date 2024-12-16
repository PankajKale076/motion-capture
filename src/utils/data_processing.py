def process_imu_data(device):
    """Process IMU data and return orientation angles."""
    imu_data = device.deviceData
    return {
        'roll': imu_data.get("AngleX", 0),
        'pitch': imu_data.get("AngleY", 0),
        'yaw': imu_data.get("AngleZ", 0)
    }

def create_data_callback(arm_controller):
    """Create a callback function for processing IMU data and controlling the arm."""
    def callback(device):
        angles = process_imu_data(device)
        print(f"Moving Arm - Roll: {angles['roll']}, Pitch: {angles['pitch']}, Yaw: {angles['yaw']}")
        arm_controller.move_with_angles(angles['roll'], angles['pitch'], angles['yaw'])
    
    return callback