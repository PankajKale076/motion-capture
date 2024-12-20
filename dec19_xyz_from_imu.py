# coding: UTF-8
import asyncio
import time
import numpy as np
from bleak import BleakClient
from filterpy.kalman import KalmanFilter

# Configuration: Define BLE device address
BLE_DEVICE_ADDRESS = "FB:B1:17:87:57:EC"  # Replace with your BLE device address

# Device Model Class
class DeviceModel:
    def __init__(self, deviceName, BLEDevice, callback_method):
        print("Initializing device model...")
        self.deviceName = deviceName  # Custom device name
        self.BLEDevice = BLEDevice  # BLE device address
        self.client = None  # BLE client
        self.writer_characteristic = None  # Write characteristic
        self.isOpen = False  # Connection state
        self.callback_method = callback_method  # Callback for data
        self.deviceData = {}  # Dictionary to store sensor data
        self.TempBytes = []  # Temporary buffer for incoming data
        self.filter_window = 5  # Size of the filter window
        self.acc_buffer = []  # Buffer for smoothing acceleration
        self.last_valid_acc = [0, 0, 0]  # Store the last valid acceleration data
        self.gravity_offset = [0, 0, 0]  # Store gravity offset
        self.is_calibrated = False
        self.calibration_samples = []
        self.num_calibration_samples = 50  # Number of samples to use for calibration
        
        # Initialize these for position and velocity tracking
        self.velocity = np.zeros(3)  # Initialize velocity as [0, 0, 0]
        self.position = np.zeros(3)  # Initialize position as [0, 0, 0]
        self.dt = 0.01  # Time step for integration (100 Hz update rate)
        self.zupt_threshold = 0.002  # Threshold for Zero Velocity Update
        self.max_position_delta = 10.0  # Limit position delta to prevent large jumps

    def calibrate_gravity(self, acc_values):
        """Calibrate the gravity offset when the sensor is stationary"""
        if not self.is_calibrated:
            self.calibration_samples.append(acc_values)
            if len(self.calibration_samples) >= self.num_calibration_samples:
                self.gravity_offset = np.mean(self.calibration_samples, axis=0)
                self.is_calibrated = True
                print(f"Gravity calibration complete. Offset: {self.gravity_offset}")
                self.calibration_samples = []

    def set(self, key, value):
        self.deviceData[key] = value

    def get(self, key):
        return self.deviceData.get(key)

    def remove(self, key):
        self.deviceData.pop(key, None)

    async def openDevice(self):
        print("Opening connection to the device...")
        try:
            async with BleakClient(self.BLEDevice, timeout=15) as client:
                self.client = client
                self.isOpen = True

                target_service_uuid = "0000ffe5-0000-1000-8000-00805f9a34fb"
                target_characteristic_uuid_read = "0000ffe4-0000-1000-8000-00805f9a34fb"
                target_characteristic_uuid_write = "0000ffe9-0000-1000-8000-00805f9a34fb"

                notify_characteristic = None

                print("Discovering services and characteristics...")
                for service in client.services:
                    if service.uuid == target_service_uuid:
                        for characteristic in service.characteristics:
                            if characteristic.uuid == target_characteristic_uuid_read:
                                notify_characteristic = characteristic
                            elif characteristic.uuid == target_characteristic_uuid_write:
                                self.writer_characteristic = characteristic
                        if notify_characteristic:
                            break

                if notify_characteristic:
                    print("Setting up notifications...")
                    await client.start_notify(notify_characteristic.uuid, self.onDataReceived)

                    if self.writer_characteristic:
                        print("Configuring sensor for 100 Hz output...")
                        await self.setOutputRate(100)

                    try:
                        while self.isOpen:
                            await asyncio.sleep(1)
                    except asyncio.CancelledError:
                        pass
                    finally:
                        await client.stop_notify(notify_characteristic.uuid)
                else:
                    print("No matching services or characteristics found.")
        except Exception as e:
            print(f"Error connecting to BLE device: {e}")

    def closeDevice(self):
        self.isOpen = False
        print("Device connection closed.")

    def onDataReceived(self, sender, data):
        tempdata = bytes.fromhex(data.hex())
        for byte in tempdata:
            self.TempBytes.append(byte)
            if len(self.TempBytes) == 1 and self.TempBytes[0] != 0x55:
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) == 2 and (self.TempBytes[1] not in [0x61, 0x71]):
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) == 20:
                self.processData(self.TempBytes)
                self.TempBytes.clear()

    def processData(self, Bytes):
        if Bytes[1] == 0x61:
            Ax = self.getSignInt16(Bytes[3] << 8 | Bytes[2]) / 32768 * 16
            Ay = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 32768 * 16
            Az = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 32768 * 16

            acc_values = np.array([Ax, Ay, Az])

            if not self.is_calibrated:
                self.calibrate_gravity(acc_values)
                return

            # Gravity correction
            acc_values -= self.gravity_offset

            # Smoothing acceleration
            smoothed_acc = self.smooth_acceleration(acc_values)
            print(f"Smoothed Acc: {smoothed_acc[0]:.3f}, {smoothed_acc[1]:.3f}, {smoothed_acc[2]:.3f}")

            # Zero-Velocity Update (ZUPT)
            movement_threshold = 0.002  # Dynamic sensitivity adjustment
            is_stationary = np.all(np.abs(smoothed_acc) < movement_threshold)

            if is_stationary:
                self.velocity *= 0  # Reset velocity
                print("IMU is stationary. Velocity reset.")
            else:
                # Update velocity using smoothed acceleration
                self.velocity += smoothed_acc * self.dt

                # Apply velocity damping to reduce drift
                velocity_damping = 0.98
                self.velocity *= velocity_damping

                # Cap velocity values to avoid unrealistic spikes
                max_velocity = 50  # mm/s
                self.velocity = np.clip(self.velocity, -max_velocity, max_velocity)

                # Update position using velocity
                position_delta = self.velocity * self.dt

                # Cap position delta to avoid large jumps
                max_position_delta = 10  # mm per cycle
                position_delta = np.clip(position_delta, -max_position_delta, max_position_delta)

                self.position += position_delta

            # Debugging: Output position
            print(f"Position: X={self.position[0]:.3f}, Y={self.position[1]:.3f}, Z={self.position[2]:.3f}")


    def smooth_acceleration(self, acc_values):
        self.acc_buffer.append(acc_values)
        if len(self.acc_buffer) > self.filter_window:
            self.acc_buffer.pop(0)
        smoothed_acc = np.mean(self.acc_buffer, axis=0)
        return smoothed_acc

    async def setOutputRate(self, rate):
        rate_mapping = {
            10: 0x0A,
            20: 0x14,
            50: 0x32,
            100: 0x64,
            200: 0xC8,
            500: 0xF4
        }
        if rate not in rate_mapping:
            print(f"Unsupported rate: {rate}")
            return
        try:
            command = [0xFF, 0xAA, 0x03, rate_mapping[rate], 0x00]
            await self.client.write_gatt_char(self.writer_characteristic.uuid, bytes(command))
            print(f"Output rate set to {rate} Hz")
        except Exception as e:
            print(f"Error setting output rate: {e}")

    @staticmethod
    def getSignInt16(num):
        if num >= 32768:
            num -= 65536
        return num

# Kalman Filter Class
class KalmanPositionTracker:
    def __init__(self):
        self.kf = self.initialize_kalman()

    def initialize_kalman(self):
        kf = KalmanFilter(dim_x=6, dim_z=3)
        kf.F = np.eye(6)
        kf.H = np.array([[1, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, 0]])
        kf.P *= 10.0
        kf.R = np.eye(3) * 0.01 # Reduce measurement noise
        kf.Q = np.eye(6) * 0.01 # Reduce process noise
        return kf

    def update(self, acc, dt):
        self.kf.predict()
        self.kf.update(acc)
        return self.kf.x[:3]

# Callback to process IMU data and track position
def process_data_callback(device):
    if not device.is_calibrated:
        print("Calibrating gravity offset... Keep the sensor still and flat...")
        return

    imu_data = device.deviceData

    acc_x = imu_data.get("AccX", 0)
    acc_y = imu_data.get("AccY", 0)
    acc_z = imu_data.get("AccZ", 0)

    acc_values = [acc_x, acc_y, acc_z]

    tracker = KalmanPositionTracker()
    dt = 0.01  # Assuming 100 Hz update rate

    # Get position from Kalman tracker
    position = tracker.update(acc_values, dt)

    # Flatten the position array
    position = position.flatten()  # Converts [[0.], [0.], [0.]] to [0., 0., 0.]

    try:
        # Unpack the values and print them
        x, y, z = position  # Now it’s a 1D array
        print(f"Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
    except Exception as e:
        # Handle unpacking or formatting errors
        print(f"Error unpacking or formatting position: {e}")

    
    

# Main logic to initialize BLE device
if __name__ == "__main__":
    # Instantiate the device model with the BLE address and callback
    device = DeviceModel("WTWitmotion", BLE_DEVICE_ADDRESS, process_data_callback)

    # Run the BLE device operation asynchronously
    try:
        print("Starting the BLE device and position tracking...")
        asyncio.run(device.openDevice())
    except KeyboardInterrupt:
        print("Stopping the BLE device connection...")
        device.closeDevice()
    except Exception as e:
        print(f"Error during operation: {e}")
