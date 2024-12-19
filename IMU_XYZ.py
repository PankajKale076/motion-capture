# Movement Control for the Xarm:
# Robotic arm uses its current position as a reference start point.
# Adjustments based on the accelerometer’s movements are applied incrementally to align the robotic arm’s TCP (Tool Center Point) with the IMU’s motion.
# Real-Time Coordination: The robotic arm continuously updates its position and orientation in real time to match the IMU’s movement.
# Absolute Movements: The code uses absolute movements relative to the robot's initial position.
# Even if the IMU is moved back to its starting orientation, the robot can track and reset its position accordingly.
# The acceleration values will not be reset to zero when the IMU stops moving.
# Instead, the last valid acceleration values will be retained and used until new valid data is received.

# coding: UTF-8
import asyncio
import time
import numpy as np
from bleak import BleakClient
from xarm.wrapper import XArmAPI

# Configuration: Define BLE device address and xArm IP here
BLE_DEVICE_ADDRESS = "CA:08:34:AF:38:7E"  # Replace with your BLE device address
ARM_IP = "192.168.1.211"  # Replace with your xArm's IP address

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

    def calibrate_gravity(self, acc_values):
        """Calibrate the gravity offset when the sensor is stationary"""
        if not self.is_calibrated:
            self.calibration_samples.append(acc_values)
            if len(self.calibration_samples) >= self.num_calibration_samples:
                # Average the samples to get the gravity offset
                self.gravity_offset = np.mean(self.calibration_samples, axis=0)
                self.is_calibrated = True
                print(f"Gravity calibration complete. Offset: {self.gravity_offset}")
                self.calibration_samples = []  # Clear the samples

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
            # Get acceleration values
            Ax = self.getSignInt16(Bytes[3] << 8 | Bytes[2]) / 32768 * 16
            Ay = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 32768 * 16
            Az = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 32768 * 16
            
            # Get angle values
            AngX = self.getSignInt16(Bytes[15] << 8 | Bytes[14]) / 32768 * 180
            AngY = self.getSignInt16(Bytes[17] << 8 | Bytes[16]) / 32768 * 180
            AngZ = self.getSignInt16(Bytes[19] << 8 | Bytes[18]) / 32768 * 180

            acc_values = [Ax, Ay, Az]
            
            if not self.is_calibrated:
                # Collect calibration samples when the sensor is stationary
                self.calibrate_gravity(acc_values)
                return
            
            # Subtract gravity offset from acceleration values
            Ax -= self.gravity_offset[0]
            Ay -= self.gravity_offset[1]
            Az -= self.gravity_offset[2]
            
            # Apply smoothing after gravity compensation
            smoothed_acc = self.smooth_acceleration([Ax, Ay, Az])
            
            print(f"Smoothed Acc: {smoothed_acc[0]:.3f}, {smoothed_acc[1]:.3f}, {smoothed_acc[2]:.3f}")

            # Update last valid acceleration data with threshold
            movement_threshold = 0.02  # Adjust this value based on testing
            if any(abs(a) > movement_threshold for a in smoothed_acc):
                self.last_valid_acc = smoothed_acc

            # Always use the last valid acceleration data
            Ax, Ay, Az = self.last_valid_acc
            
            print(f"Last Valid Acc: {Ax:.3f}, {Ay:.3f}, {Az:.3f}")

            # Store processed values
            self.set("AccX", round(Ax, 3))
            self.set("AccY", round(Ay, 3))
            self.set("AccZ", round(Az, 3))
            self.set("AngleX", round(AngX, 3))
            self.set("AngleY", round(AngY, 3))
            self.set("AngleZ", round(AngZ, 3))

            # Call the callback method with updated values
            self.callback_method(self)

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

# Arm Control Class
class ArmController:
    def __init__(self, arm_ip):
        self.arm = XArmAPI(arm_ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1)
        self.arm.set_state(0)
        self.safety_bounds = {
            "x": (-500, 500),  # mm
            "y": (-500, 500),  # mm
            "z": (0, 500),      # mm
            "roll": (-180, 180),
            "pitch": (-90, 90),
            "yaw": (-180, 180)
        }
        self.initial_position = self.arm.get_position(is_radian=False)  # Get initial position

    def move_with_sensor(self, delta_x, delta_y, delta_z, roll, pitch, yaw):
        # Apply non-linear scaling for movements
        delta_x = self.non_linear_scaling(delta_x)
        delta_y = self.non_linear_scaling(delta_y)
        delta_z = self.non_linear_scaling(delta_z)

        # Get current position
        code, curr_pos = self.arm.get_position()
        if code != 0:
            print(f"Error getting position: {code}")
            return
        
        # Extract the position values we need
        curr_x = float(curr_pos[0])
        curr_y = float(curr_pos[1])
        curr_z = float(curr_pos[2])

        # Map deltas to real-world robot movement, ensuring safety bounds
        target_x = max(self.safety_bounds["x"][0], min(curr_x + delta_x, self.safety_bounds["x"][1]))
        target_y = max(self.safety_bounds["y"][0], min(curr_y + delta_y, self.safety_bounds["y"][1]))
        target_z = max(self.safety_bounds["z"][0], min(curr_z + delta_z, self.safety_bounds["z"][1]))

        # Ensure rotation values are within bounds
        roll = max(self.safety_bounds["roll"][0], min(roll, self.safety_bounds["roll"][1]))
        pitch = max(self.safety_bounds["pitch"][0], min(pitch, self.safety_bounds["pitch"][1]))
        yaw = max(self.safety_bounds["yaw"][0], min(yaw, self.safety_bounds["yaw"][1]))

        try:
            self.arm.set_servo_cartesian([target_x, target_y, target_z, roll, pitch, yaw], is_radian=False)
            print(f"Moving arm to: X: {target_x}, Y: {target_y}, Z: {target_z}, Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        except Exception as e:
            print(f"Error moving the arm: {e}")

    def non_linear_scaling(self, value, scale_factor=1.5):
        """
        Apply non-linear scaling to movement values to provide finer control
        :param value: input value to scale
        :param scale_factor: exponential factor for scaling
        :return: scaled value
        """
        return float(np.sign(value)) * (abs(float(value)) ** scale_factor)

# Callback to process IMU data and move the robotic arm
def process_data_callback(device):
    if not device.is_calibrated:
        print("Calibrating gravity offset... Keep the sensor still and flat...")
        return

    imu_data = device.deviceData
    
    # Add a deadzone to filter out tiny movements
    deadzone = 0.001
    
    # Get acceleration values with deadzone
    acc_x = imu_data.get("AccX", 0)
    acc_y = imu_data.get("AccY", 0)
    acc_z = imu_data.get("AccZ", 0)

    print(f"deadzone acc_x: {acc_x:.3f}, acc_y: {acc_y:.3f}, acc_z: {acc_z:.3f}")

    # Get angle values
    roll = imu_data.get("AngleX", 0)
    pitch = imu_data.get("AngleY", 0)
    yaw = imu_data.get("AngleZ", 0)

    # Only move and print if there's actual movement detected
    if abs(acc_x) > 0 or abs(acc_y) > 0 or abs(acc_z) > 0:
        print(f"Moving Arm - Delta X: {acc_x:.3f} mm, Delta Y: {acc_y:.3f} mm, Delta Z: {acc_z:.3f} mm, "
              f"Roll: {roll:.3f}, Pitch: {pitch:.3f}, Yaw: {yaw:.3f}")
        arm_controller.move_with_sensor(acc_x, acc_y, acc_z, roll, pitch, yaw)
        
# Main logic to initialize the BLE device and robotic arm
if __name__ == "__main__":
    arm_controller = ArmController(ARM_IP)
    device = DeviceModel("WTWitmotion", BLE_DEVICE_ADDRESS, process_data_callback)

    try:
        asyncio.run(device.openDevice())
    except KeyboardInterrupt:
        device.closeDevice()
    except Exception as e:
        print(f"Error during operation: {e}")
