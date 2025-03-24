# coding: UTF-8
import asyncio
import time
import numpy as np
from bleak import BleakClient, BleakError
from xarm.wrapper import XArmAPI


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

        # Calibration values
        self.calibration_offsets = {"AngleX": 0, "AngleY": 0, "AngleZ": 0}

        # Filter parameters
        self.filter_alpha = 0.8  # Smoothing factor (0-1), higher = less smoothing
        self.prev_angles = {"AngleX": 0, "AngleY": 0, "AngleZ": 0}

        # Dead zone (in degrees) - movements smaller than this won't be reported
        self.dead_zone = 0.5

    def set(self, key, value):
        self.deviceData[key] = value

    def get(self, key):
        return self.deviceData.get(key)

    def remove(self, key):
        self.deviceData.pop(key, None)

    def calibrate_imu(self):
        """Store current orientation as the zero position"""
        for axis in ["AngleX", "AngleY", "AngleZ"]:
            if axis in self.deviceData:
                self.calibration_offsets[axis] = self.deviceData[axis]
                print(
                    f"Calibration offset for {axis}: {self.calibration_offsets[axis]}"
                )

    async def openDevice(self):
        print("Opening connection to the device...")
        while True:  # Reconnection loop
            try:
                async with BleakClient(self.BLEDevice, timeout=15) as client:
                    self.client = client
                    self.isOpen = True
                    print(f"Connected to {self.deviceName}")

                    target_service_uuid = "0000ffe5-0000-1000-8000-00805f9a34fb"
                    target_characteristic_uuid_read = (
                        "0000ffe4-0000-1000-8000-00805f9a34fb"
                    )
                    target_characteristic_uuid_write = (
                        "0000ffe9-0000-1000-8000-00805f9a34fb"
                    )

                    notify_characteristic = None

                    print("Discovering services and characteristics...")
                    for service in client.services:
                        if service.uuid == target_service_uuid:
                            for characteristic in service.characteristics:
                                if (
                                    characteristic.uuid
                                    == target_characteristic_uuid_read
                                ):
                                    notify_characteristic = characteristic
                                elif (
                                    characteristic.uuid
                                    == target_characteristic_uuid_write
                                ):
                                    self.writer_characteristic = characteristic
                            if notify_characteristic:
                                break

                    if not notify_characteristic or not self.writer_characteristic:
                        print("Required services or characteristics not found.")
                        self.isOpen = False
                        await asyncio.sleep(2)  # Wait before reconnection attempt
                        continue

                    print("Setting up notifications...")
                    await client.start_notify(
                        notify_characteristic.uuid, self.onDataReceived
                    )

                    print("Configuring sensor for 100 Hz output...")
                    await self.setOutputRate(100)

                    try:
                        while self.isOpen:
                            await asyncio.sleep(1)
                    except asyncio.CancelledError:
                        print("Operation cancelled.")
                        break
                    finally:
                        try:
                            await client.stop_notify(notify_characteristic.uuid)
                            print("Notifications stopped.")
                        except:
                            pass
            except BleakError as e:
                print(f"BLE Error: {e}")
                self.isOpen = False
                await asyncio.sleep(5)  # Wait before reconnection attempt
                print("Attempting to reconnect...")
            except Exception as e:
                print(f"Unexpected error: {e}")
                self.isOpen = False
                await asyncio.sleep(5)  # Wait before reconnection attempt
                print("Attempting to reconnect...")

            if not self.isOpen:
                continue  # Try to reconnect
            else:
                break  # Exit if the connection was closed properly

    def closeDevice(self):
        self.isOpen = False
        print("Device connection closed.")

    def onDataReceived(self, sender, data):
        try:
            tempdata = bytes.fromhex(data.hex())
            for byte in tempdata:
                self.TempBytes.append(byte)

                # Reset if first byte is not the header byte
                if len(self.TempBytes) == 1 and self.TempBytes[0] != 0x55:
                    del self.TempBytes[0]
                    continue

                # Reset if second byte is not one of the expected types
                if len(self.TempBytes) == 2 and (self.TempBytes[1] not in [0x61, 0x71]):
                    del self.TempBytes[0]
                    continue

                # When we have a complete packet
                if len(self.TempBytes) == 20:
                    self.processData(self.TempBytes)
                    self.TempBytes.clear()
        except Exception as e:
            print(f"Error processing received data: {e}")
            self.TempBytes.clear()

    def processData(self, Bytes):
        try:
            if Bytes[1] == 0x61:
                # Process raw IMU data
                AngX = self.getSignInt16(Bytes[15] << 8 | Bytes[14]) / 32768 * 180
                AngY = self.getSignInt16(Bytes[17] << 8 | Bytes[16]) / 32768 * 180
                AngZ = self.getSignInt16(Bytes[19] << 8 | Bytes[18]) / 32768 * 180

                # Apply calibration offsets
                AngX -= self.calibration_offsets["AngleX"]
                AngY -= self.calibration_offsets["AngleY"]
                AngZ -= self.calibration_offsets["AngleZ"]

                # Apply low-pass filter for smoothing
                AngX = self.apply_filter("AngleX", AngX)
                AngY = self.apply_filter("AngleY", AngY)
                AngZ = self.apply_filter("AngleZ", AngZ)

                # Apply dead zone
                if (
                    abs(AngX - self.get("AngleX") if self.get("AngleX") else AngX)
                    < self.dead_zone
                ):
                    AngX = self.get("AngleX") if self.get("AngleX") else AngX
                if (
                    abs(AngY - self.get("AngleY") if self.get("AngleY") else AngY)
                    < self.dead_zone
                ):
                    AngY = self.get("AngleY") if self.get("AngleY") else AngY
                if (
                    abs(AngZ - self.get("AngleZ") if self.get("AngleZ") else AngZ)
                    < self.dead_zone
                ):
                    AngZ = self.get("AngleZ") if self.get("AngleZ") else AngZ

                # Store filtered values
                self.set("AngleX", round(AngX, 3))
                self.set("AngleY", round(AngY, 3))
                self.set("AngleZ", round(AngZ, 3))

                # Call the callback with new data
                self.callback_method(self)
        except Exception as e:
            print(f"Error in processData: {e}")

    def apply_filter(self, key, new_value):
        """Apply a simple low-pass filter for smoothing"""
        if key in self.prev_angles:
            filtered = (
                self.filter_alpha * new_value
                + (1 - self.filter_alpha) * self.prev_angles[key]
            )
            self.prev_angles[key] = filtered
            return filtered
        else:
            self.prev_angles[key] = new_value
            return new_value

    async def setOutputRate(self, rate):
        """Set the IMU output data rate"""
        rate_mapping = {10: 0x0A, 20: 0x14, 50: 0x32, 100: 0x64, 200: 0xC8, 500: 0xF4}
        if rate not in rate_mapping:
            print(f"Unsupported rate: {rate}")
            return False

        try:
            command = [0xFF, 0xAA, 0x03, rate_mapping[rate], 0x00]
            await self.client.write_gatt_char(
                self.writer_characteristic.uuid, bytes(command)
            )
            print(f"Output rate set to {rate} Hz")
            return True
        except Exception as e:
            print(f"Error setting output rate: {e}")
            return False

    @staticmethod
    def getSignInt16(num):
        if num >= 32768:
            num -= 65536
        return num


# Arm Control Class
class ArmController:
    def __init__(self, arm_ip):
        print(f"Connecting to arm at {arm_ip}...")
        self.arm = XArmAPI(arm_ip)

        # Enable the arm and set mode
        code, state = self.arm.get_state()
        print(f"Arm state: {state}")

        code = self.arm.motion_enable(enable=True)
        if code != 0:
            print(f"Failed to enable motion, error code: {code}")

        code = self.arm.set_mode(1)  # Set to position control mode
        if code != 0:
            print(f"Failed to set mode, error code: {code}")

        code = self.arm.set_state(0)  # Set to ready state
        if code != 0:
            print(f"Failed to set state, error code: {code}")

        # Get current position
        code, self.curr_pos = self.arm.get_position()
        if code != 0:
            print(f"Failed to get position, error code: {code}")
            self.curr_pos = [0, 0, 0, 0, 0, 0]

        # Define joint limits
        self.joint_limits = {
            "roll": {"min": -180, "max": 180},  # Roll limits
            "pitch": {"min": -90, "max": 90},  # Pitch limits
            "yaw": {"min": -180, "max": 180},  # Yaw limits
        }

        # Last command time for rate limiting
        self.last_command_time = 0
        self.min_command_interval = 0.05  # 50ms between commands (20Hz max)

        print("Arm controller initialized.")

    def constrain_angle(self, angle, limit_type):
        """Constrain angle to within joint limits"""
        min_val = self.joint_limits[limit_type]["min"]
        max_val = self.joint_limits[limit_type]["max"]
        return max(min_val, min(angle, max_val))

    def move_with_angles(self, roll, pitch, yaw):
        """Move the arm to a specified orientation while respecting joint limits"""
        # Apply joint limits
        roll_constrained = self.constrain_angle(roll, "roll")
        pitch_constrained = self.constrain_angle(pitch, "pitch")
        yaw_constrained = self.constrain_angle(yaw, "yaw")

        # Check if any angles were constrained
        if (
            roll != roll_constrained
            or pitch != pitch_constrained
            or yaw != yaw_constrained
        ):
            print(f"Warning: Angles constrained to joint limits")
            print(f"  Roll: {roll} -> {roll_constrained}")
            print(f"  Pitch: {pitch} -> {pitch_constrained}")
            print(f"  Yaw: {yaw} -> {yaw_constrained}")

        # Rate limiting
        current_time = time.time()
        if current_time - self.last_command_time < self.min_command_interval:
            return
        self.last_command_time = current_time

        # Send command to the arm
        code = self.arm.set_servo_cartesian(
            [
                self.curr_pos[0],
                self.curr_pos[1],
                self.curr_pos[2],
                roll_constrained,
                pitch_constrained,
                yaw_constrained,
            ],
            is_radian=False,
            wait=False,
        )

        if code != 0:
            print(f"Arm movement error, code: {code}")

    def emergency_stop(self):
        """Emergency stop the arm"""
        code = self.arm.emergency_stop()
        print(f"Emergency stop triggered. Result code: {code}")

    def disconnect(self):
        """Properly disconnect from the arm"""
        code = self.arm.disconnect()
        print(f"Arm disconnected. Result code: {code}")


# Callback to process IMU data and move the robotic arm
def process_data_callback(device):
    imu_data = device.deviceData
    roll = imu_data.get("AngleX", 0)
    pitch = imu_data.get("AngleY", 0)
    yaw = imu_data.get("AngleZ", 0)

    # Move the arm with filtered and constrained angles
    arm_controller.move_with_angles(roll, pitch, yaw)


async def main():
    BLE_DEVICE_ADDRESS = "CA:08:34:AF:38:7E"
    ARM_IP = "192.168.1.211"  # Replace with your xArm's IP address

    global arm_controller
    arm_controller = ArmController(ARM_IP)
    device = DeviceModel("WTWitmotion", BLE_DEVICE_ADDRESS, process_data_callback)

    # Setup keyboard commands
    async def handle_keyboard_input():
        print("Commands: 'c' to calibrate IMU, 'q' to quit, 'e' for emergency stop")

        while True:
            cmd = await asyncio.to_thread(input)
            if cmd.lower() == "c":
                print("Calibrating IMU...")
                device.calibrate_imu()
            elif cmd.lower() == "e":
                print("Emergency stop triggered!")
                arm_controller.emergency_stop()
            elif cmd.lower() == "q":
                print("Quitting...")
                device.closeDevice()
                arm_controller.disconnect()
                break

    try:
        # Run the keyboard input handler and device connection concurrently
        await asyncio.gather(handle_keyboard_input(), device.openDevice())
    except KeyboardInterrupt:
        print("Keyboard interrupt detected.")
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        device.closeDevice()
        arm_controller.disconnect()
        print("Program terminated.")


if __name__ == "__main__":
    asyncio.run(main())
