# coding: UTF-8
import asyncio
import time
import numpy as np
from bleak import BleakClient
from xarm.wrapper import XArmAPI

# Configuration: Define BLE device address and xArm IP here
BLE_DEVICE_ADDRESS = "FB:B1:17:87:57:EC"  # Replace with your BLE device address
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
        self.gravity_comp = [0, 0, 9.81]  # Gravity compensation placeholder

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
            Ax = self.getSignInt16(Bytes[3] << 8 | Bytes[2]) / 32768 * 16 * 9.81  # m/s^2
            Ay = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 32768 * 16 * 9.81
            Az = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 32768 * 16 * 9.81
            AngX = self.getSignInt16(Bytes[15] << 8 | Bytes[14]) / 32768 * 180
            AngY = self.getSignInt16(Bytes[17] << 8 | Bytes[16]) / 32768 * 180
            AngZ = self.getSignInt16(Bytes[19] << 8 | Bytes[18]) / 32768 * 180

            smoothed_acc = self.smooth_acceleration([Ax, Ay, Az])

            # Subtract gravity component from Z
            smoothed_acc[2] -= self.gravity_comp[2]

            # Update only when movement is significant
            threshold = 0.1 # Movement threshold
            if any(abs(a) > threshold for a in smoothed_acc):
                self.set("AccX", round(smoothed_acc[0], 3))
                self.set("AccY", round(smoothed_acc[1], 3))
                self.set("AccZ", round(smoothed_acc[2], 3))
                self.set("AngleX", round(AngX, 3))
                self.set("AngleY", round(AngY, 3))
                self.set("AngleZ", round(AngZ, 3))

                self.callback_method(self)

    def smooth_acceleration(self, acc_values):
        self.acc_buffer.append(acc_values)
        if len(self.acc_buffer) > self.filter_window:
            self.acc_buffer.pop(0)
        return np.mean(self.acc_buffer, axis=0)

    async def setOutputRate(self, rate):
        rate_mapping = {10: 0x0A, 20: 0x14, 50: 0x32, 100: 0x64}
        if rate not in rate_mapping:
            print(f"Unsupported rate: {rate}")
            return
        try:
            command = [0xFF, 0xAA, 0x03, rate_mapping[rate], 0x00]
            await self.client.write_gatt_char(self.writer_characteristic.uuid, bytes(command))
        except Exception as e:
            print(f"Error setting output rate: {e}")

    @staticmethod
    def getSignInt16(num):
        return num - 65536 if num >= 32768 else num

# Callback to process IMU data and move the robotic arm
def process_data_callback(device):
    imu_data = device.deviceData

    acc_x = imu_data.get("AccX", 0) * 10  # Convert m/s^2 to mm
    acc_y = imu_data.get("AccY", 0) * 10
    acc_z = imu_data.get("AccZ", 0) * 10

    roll = imu_data.get("AngleX", 0)
    pitch = imu_data.get("AngleY", 0)
    yaw = imu_data.get("AngleZ", 0)

    # Threshold to ignore noise
    threshold = 1.0
    if abs(acc_x) > threshold or abs(acc_y) > threshold or abs(acc_z) > threshold:
        print(f"Moving Robot - X: {acc_x:.2f} mm, Y: {acc_y:.2f} mm, Z: {acc_z:.2f} mm, Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
        # Simulated robot movement or integration to xArm

# Main logic to initialize the BLE device and robotic arm
if __name__ == "__main__":
    device = DeviceModel("WTWitmotion", BLE_DEVICE_ADDRESS, process_data_callback)
    try:
        asyncio.run(device.openDevice())
    except KeyboardInterrupt:
        device.closeDevice()
    except Exception as e:
        print(f"Error during operation: {e}")
