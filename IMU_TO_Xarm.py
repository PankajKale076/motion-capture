# coding: UTF-8
import asyncio
import time
from bleak import BleakClient
from xarm.wrapper import XArmAPI
import numpy as np
from datetime import datetime

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

    def set(self, key, value):
        self.deviceData[key] = value

    def get(self, key):
        return self.deviceData.get(key)

    def remove(self, key):
        self.deviceData.pop(key, None)

    async def openDevice(self):
        print("Opening connection to the device...")
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
            AngX = self.getSignInt16(Bytes[15] << 8 | Bytes[14]) / 32768 * 180
            AngY = self.getSignInt16(Bytes[17] << 8 | Bytes[16]) / 32768 * 180
            AngZ = self.getSignInt16(Bytes[19] << 8 | Bytes[18]) / 32768 * 180
            self.set("AccX", round(Ax, 3))
            self.set("AccY", round(Ay, 3))
            self.set("AccZ", round(Az, 3))
            self.set("AngleX", round(AngX, 3))
            self.set("AngleY", round(AngY, 3))
            self.set("AngleZ", round(AngZ, 3))
            self.callback_method(self)

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
        command = [0xFF, 0xAA, 0x03, rate_mapping[rate], 0x00]
        await self.client.write_gatt_char(self.writer_characteristic.uuid, bytes(command))
        print(f"Output rate set to {rate} Hz")

    @staticmethod
    def getSignInt16(num):
        if num >= 32768:
            num -= 65536
        return num

# Arm Control Class

class PositionTracker:
    def __init__(self):
        # Initialize state variables
        self.last_update = datetime.now()
        self.velocity = np.zeros(3)  # Vx, Vy, Vz
        self.position = np.zeros(3)  # X, Y, Z
        self.last_accel = np.zeros(3)  # Last acceleration reading
        
        # Constants for filtering
        self.accel_threshold = 0.03  # Minimum acceleration to consider (helps reduce drift)
        self.accel_change_threshhold = 0.01
        
        # Scale factors for converting accelerometer units to m/s²
        self.accel_scale = 9.81  # Assuming accelerometer values are in g's
        
        # Position scaling for robot workspace (adjust based on your robot's workspace)
        self.position_scale = 5  # Scale factor to map position to robot coordinates
        self.position_limits = np.array([
            [-300, 300],  # X limits in mm
            [-300, 300],  # Y limits in mm
            [0, 500]      # Z limits in mm
        ])

    def update(self, accel_data):
        """
        Update position based on new accelerometer readings
        accel_data: dict containing 'AccX', 'AccY', 'AccZ' in g's
        Returns: tuple of (x, y, z) positions in robot coordinates
        """
        # Get time delta
        current_time = datetime.now()
        dt = (current_time - self.last_update).total_seconds()
        self.last_update = current_time
        
        # Convert acceleration data to numpy array in m/s²
        accel = np.array([
            accel_data['AccX'],
            accel_data['AccY'],
            accel_data['AccZ'] + 1  # Add 1 to Z to remove gravity
        ]) * self.accel_scale
        
        # Check for significant change in acceleration
        accel_change = np.abs(accel - self.last_accel)
        is_moving = np.any(accel_change > self.accel_change_threshhold)
        
        # Update last acceleration
        self.last_accel = accel.copy()
        
        if not is_moving:
            self.velocity = np.zeros(3)
            return tuple(self.position * self.position_scale)
        
        # Only integrate acceleration if moving
        if np.any(np.abs(accel) > self.accel_threshold):
            self.velocity += accel * dt
            self.position += self.velocity * dt
            
            # Apply position limits
            self.position = np.clip(self.position, self.position_limits[:, 0], self.position_limits[:, 1])
            
        return tuple(self.position * self.position_scale)
class ArmController:
    def __init__(self, arm_ip):
        self.arm = XArmAPI(arm_ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.position_tracker = PositionTracker()
        
        # Store initial position
        code, self.initial_position = self.arm.get_position()

    def move_with_sensor_data(self, sensor_data):
        """
        Move the arm based on sensor data
        sensor_data: dict containing accelerometer and angle data
        """
        # Get position offset from accelerometer data
        position_offset = self.position_tracker.update({
            'AccX': sensor_data['AccX'],
            'AccY': sensor_data['AccY'],
            'AccZ': sensor_data['AccZ']
        })
        
        # Get orientation from angle data
        roll = sensor_data['AngleX']
        pitch = sensor_data['AngleY']
        yaw = sensor_data['AngleZ']
        
        # Calculate new absolute position
        new_position = [
            self.initial_position[0] + position_offset[0],
            self.initial_position[1] + position_offset[1],
            self.initial_position[2] + position_offset[2],
            roll, pitch, yaw
        ]
        
        print(f"Moving to position: {new_position}")
        
        # Move robot to new position
        self.arm.set_servo_cartesian(new_position, is_radian=False, wait=False)

# Modified callback function
def process_data_callback(device):
    sensor_data = device.deviceData
    arm_controller.move_with_sensor_data(sensor_data)

if __name__ == "__main__":
    BLE_DEVICE_ADDRESS = "CA:08:34:AF:38:7E"
    ARM_IP = "192.168.1.211"

    arm_controller = ArmController(ARM_IP)
    device = DeviceModel("WTWitmotion", BLE_DEVICE_ADDRESS, process_data_callback)

    try:
        asyncio.run(device.openDevice())
    except KeyboardInterrupt:
        device.closeDevice()
