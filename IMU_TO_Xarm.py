# coding: UTF-8
import asyncio
import time
from bleak import BleakClient
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
            AngX = self.getSignInt16(Bytes[15] << 8 | Bytes[14]) / 32768 * 180
            AngY = self.getSignInt16(Bytes[17] << 8 | Bytes[16]) / 32768 * 180
            AngZ = self.getSignInt16(Bytes[19] << 8 | Bytes[18]) / 32768 * 180
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
class ArmController:
    def __init__(self, arm_ip):
        self.arm = XArmAPI(arm_ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

    def move_with_angles(self, roll, pitch, yaw):
        roll = roll % 360
        pitch = pitch % 360
        yaw = yaw % 360
        self.arm.set_servo_cartesian([0, 0, 0, roll, pitch, yaw], is_radian=False)

# Callback to process IMU data and move the robotic arm
def process_data_callback(device):
    imu_data = device.deviceData
    roll = imu_data.get("AngleX", 0)
    pitch = imu_data.get("AngleY", 0)
    yaw = imu_data.get("AngleZ", 0)

    print(f"Moving Arm - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
    arm_controller.move_with_angles(roll, pitch, yaw)

if __name__ == "__main__":
    BLE_DEVICE_ADDRESS = "FB:B1:17:87:57:EC"
    ARM_IP = "192.168.1.100"  # Replace with your xArm's IP address

    arm_controller = ArmController(ARM_IP)
    device = DeviceModel("WTWitmotion", BLE_DEVICE_ADDRESS, process_data_callback)

    try:
        asyncio.run(device.openDevice())
    except KeyboardInterrupt:
        device.closeDevice()
