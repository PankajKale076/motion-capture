import asyncio
import time
import numpy as np
from bleak import BleakClient

# Configuration
BLE_DEVICE_ADDRESS = "FB:B1:17:87:57:EC"

# Robot workspace boundaries (mm)
X_MIN, X_MAX = -500, 500  # ±500mm in X
Y_MIN, Y_MAX = -500, 500  # ±500mm in Y
Z_MIN, Z_MAX = 0, 500     # 0-500mm in Z

class MotionTracker:
    def __init__(self):
        # Calibration variables
        self.is_calibrated = False
        self.calibration_samples = []
        self.gravity_offset = np.zeros(3)
        
        # State variables for integration
        self.velocity = np.zeros(3)  # Current velocity in m/s
        self.position = np.zeros(3)  # Current position in mm
        self.last_time = time.time()
        self.last_acceleration = np.zeros(3)
        
        # Buffer for acceleration smoothing
        self.accel_buffer = []
        self.buffer_size = 10
        
        # Motion detection
        self.is_moving = False
        self.stable_count = 0
        self.STABILITY_COUNT_THRESHOLD = 10
        
        # Integration constants
        self.ACCEL_THRESHOLD = 0.02  # Minimum acceleration to consider (g)
        self.VELOCITY_THRESHOLD = 0.01  # Minimum velocity to consider (m/s)
        self.VELOCITY_DECAY = 0.95  # Velocity decay factor
        self.MM_PER_METER = 1000  # Conversion factor from meters to millimeters
        
    def calibrate(self, acceleration):
        """Calibrate to remove gravity and sensor bias."""
        if not self.is_calibrated:
            self.calibration_samples.append(acceleration)
            if len(self.calibration_samples) >= 100:
                self.gravity_offset = np.mean(self.calibration_samples, axis=0)
                self.is_calibrated = True
                print("Calibration complete. Ready for movement tracking.")
                return True
            if len(self.calibration_samples) % 10 == 0:
                print(f"Calibrating... {len(self.calibration_samples)}/100")
            return False
        return True

    def smooth_acceleration(self, acceleration):
        """Apply moving average smoothing to acceleration data."""
        self.accel_buffer.append(acceleration)
        if len(self.accel_buffer) > self.buffer_size:
            self.accel_buffer.pop(0)
        return np.mean(self.accel_buffer, axis=0)

    def integrate_acceleration(self, acceleration, dt):
        """First integration: Acceleration to Velocity."""
        # Convert acceleration from g to m/s²
        acc_ms2 = acceleration * 9.81
        
        # Perform the first integration to get velocity (m/s)
        velocity_change = acc_ms2 * dt
        self.velocity += velocity_change
        
        # Apply velocity decay to prevent drift
        self.velocity *= self.VELOCITY_DECAY
        
        # Zero out small velocities to prevent drift
        mask = np.abs(self.velocity) < self.VELOCITY_THRESHOLD
        self.velocity[mask] = 0
        
        return self.velocity

    def integrate_velocity(self, velocity, dt):
        """Second integration: Velocity to Position."""
        # Perform the second integration to get position change in meters
        position_change_m = velocity * dt
        
        # Convert to millimeters
        position_change_mm = position_change_m * self.MM_PER_METER
        
        # Update position
        self.position += position_change_mm
        
        # Apply workspace limits
        self.position[0] = np.clip(self.position[0], X_MIN, X_MAX)
        self.position[1] = np.clip(self.position[1], Y_MIN, Y_MAX)
        self.position[2] = np.clip(self.position[2], Z_MIN, Z_MAX)
        
        return self.position

    def detect_motion(self, smoothed_acc):
        """Detect if the IMU is in motion."""
        acc_magnitude = np.linalg.norm(smoothed_acc)
        
        if acc_magnitude < self.ACCEL_THRESHOLD:
            self.stable_count += 1
        else:
            self.stable_count = 0
            self.is_moving = True

        if self.stable_count >= self.STABILITY_COUNT_THRESHOLD:
            self.is_moving = False
            self.velocity = np.zeros(3)  # Reset velocity when stable

        return self.is_moving

    def update_position(self, acceleration):
        """Update position using double integration of acceleration."""
        if not self.is_calibrated:
            return np.zeros(3)
        
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Remove gravity offset and apply smoothing
        acc_corrected = acceleration - self.gravity_offset
        smoothed_acc = self.smooth_acceleration(acc_corrected)
        
        # Detect motion state
        is_moving = self.detect_motion(smoothed_acc)
        
        if not is_moving:
            return self.position
        
        # First integration: acceleration to velocity
        velocity = self.integrate_acceleration(smoothed_acc, dt)
        
        # Second integration: velocity to position
        position = self.integrate_velocity(velocity, dt)
        
        return position

class IMUProcessor:
    def __init__(self, device_name, ble_address):
        self.device_name = device_name
        self.ble_address = ble_address
        self.client = None
        self.tracker = MotionTracker()
        
    async def connect(self):
        """Connect to the IMU device."""
        try:
            async with BleakClient(self.ble_address) as client:
                self.client = client
                print(f"Connected to {self.device_name}")
                print("Hold the IMU still for calibration...")
                await self.setup_notifications(client)
        except Exception as e:
            print(f"Connection error: {e}")
    
    async def setup_notifications(self, client):
        """Setup BLE notifications."""
        char_uuid = "0000ffe4-0000-1000-8000-00805f9a34fb"
        await client.start_notify(char_uuid, self.process_imu_data)
        try:
            while True:
                await asyncio.sleep(0.01)
        except asyncio.CancelledError:
            await client.stop_notify(char_uuid)
    
    def process_imu_data(self, sender, data):
        """Process incoming IMU data."""
        try:
            # Extract acceleration data
            acc_x = int.from_bytes(data[2:4], byteorder="little", signed=True) / 32768 * 16
            acc_y = int.from_bytes(data[4:6], byteorder="little", signed=True) / 32768 * 16
            acc_z = int.from_bytes(data[6:8], byteorder="little", signed=True) / 32768 * 16
            
            acceleration = np.array([acc_x, acc_y, acc_z])
            
            # Handle calibration
            if not self.tracker.calibrate(acceleration):
                return
            
            # Update position through double integration
            position = self.tracker.update_position(acceleration)
            
            # Print position and motion state
            print("Robot Position - X: {:.1f}, Y: {:.1f}, Z: {:.1f} - {}".format(
                position[0], position[1], position[2],
                "Moving" if self.tracker.is_moving else "Stationary"
            ))
                
        except Exception as e:
            print(f"Error processing IMU data: {e}")
            import traceback
            traceback.print_exc()

async def main():
    """Main function to run the IMU processor."""
    imu = IMUProcessor("WT901BLECL", BLE_DEVICE_ADDRESS)
    try:
        await imu.connect()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if imu.client:
            await imu.client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())