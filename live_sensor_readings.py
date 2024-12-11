# coding: UTF-8
import asyncio
import time
from bleak import BleakClient

ble_address = "CA:08:34:AF:38:7E"

# Device Model Class
class DeviceModel:
    # Initialization
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

    # Store device data
    def set(self, key, value):
        self.deviceData[key] = value

    # Retrieve device data
    def get(self, key):
        return self.deviceData.get(key)

    # Remove device data
    def remove(self, key):
        self.deviceData.pop(key, None)

    # Open BLE connection and set up notifications
    async def openDevice(self):
        print("Opening connection to the device...")
        # Connect to the BLE device
        async with BleakClient(self.BLEDevice, timeout=15) as client:
            self.client = client
            self.isOpen = True

            # Service and characteristic UUIDs for the WTWitmotion device
            target_service_uuid = "0000ffe5-0000-1000-8000-00805f9a34fb"
            target_characteristic_uuid_read = "0000ffe4-0000-1000-8000-00805f9a34fb"
            target_characteristic_uuid_write = "0000ffe9-0000-1000-8000-00805f9a34fb"

            notify_characteristic = None

            # Discover services and characteristics
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

            # Enable notifications for data reception
            if notify_characteristic:
                print("Setting up notifications...")
                await client.start_notify(notify_characteristic.uuid, self.onDataReceived)

                # Send a command to set the sensor to 100 Hz data output
                if self.writer_characteristic:
                    print("Configuring sensor for 100 Hz output...")
                    await self.setOutputRate(100)

                # Keep the connection open
                try:
                    while self.isOpen:
                        await asyncio.sleep(1)
                except asyncio.CancelledError:
                    pass
                finally:
                    # Stop notifications when closing
                    await client.stop_notify(notify_characteristic.uuid)
            else:
                print("No matching services or characteristics found.")

    # Close BLE connection
    def closeDevice(self):
        self.isOpen = False
        print("Device connection closed.")

    # Handle incoming data from the device
    def onDataReceived(self, sender, data):
        tempdata = bytes.fromhex(data.hex())
        for byte in tempdata:
            self.TempBytes.append(byte)
            # Validate and process data
            if len(self.TempBytes) == 1 and self.TempBytes[0] != 0x55:
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) == 2 and (self.TempBytes[1] not in [0x61, 0x71]):
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) == 20:
                self.processData(self.TempBytes)
                self.TempBytes.clear()

    # Process and parse data packet
    def processData(self, Bytes):
        if Bytes[1] == 0x61:  # Accelerometer, Gyroscope, and Angle data
            Ax = self.getSignInt16(Bytes[3] << 8 | Bytes[2]) / 32768 * 16
            Ay = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 32768 * 16
            Az = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 32768 * 16
            Gx = self.getSignInt16(Bytes[9] << 8 | Bytes[8]) / 32768 * 2000
            Gy = self.getSignInt16(Bytes[11] << 8 | Bytes[10]) / 32768 * 2000
            Gz = self.getSignInt16(Bytes[13] << 8 | Bytes[12]) / 32768 * 2000
            AngX = self.getSignInt16(Bytes[15] << 8 | Bytes[14]) / 32768 * 180
            AngY = self.getSignInt16(Bytes[17] << 8 | Bytes[16]) / 32768 * 180
            AngZ = self.getSignInt16(Bytes[19] << 8 | Bytes[18]) / 32768 * 180
            self.set("AccX", round(Ax, 3))
            self.set("AccY", round(Ay, 3))
            self.set("AccZ", round(Az, 3))
            self.set("GyroX", round(Gx, 3))
            self.set("GyroY", round(Gy, 3))
            self.set("GyroZ", round(Gz, 3))
            self.set("AngleX", round(AngX, 3))
            self.set("AngleY", round(AngY, 3))
            self.set("AngleZ", round(AngZ, 3))
            self.callback_method(self)  # Call the callback method with updated data

    # Set sensor output rate
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
        # Construct the write command
        command = [0xFF, 0xAA, 0x03, 0x64, 0x00]
        await self.client.write_gatt_char(self.writer_characteristic.uuid, bytes(command))
        print(f"Output rate set to {rate} Hz")

    # Convert unsigned int16 to signed int16
    @staticmethod
    def getSignInt16(num):
        if num >= 32768:
            num -= 65536
        return num


# Keep track of timestamps
last_timestamp = None
readings_count = 0
start_time = None

# Callback method to process data
def process_data_callback(device):
    global last_timestamp, readings_count, start_time
    
    # Current time in seconds
    current_time = time.time()

    # Initialize start time on first reading
    if start_time is None:
        start_time = current_time
    
    # Count the number of readings
    readings_count += 1

    # Calculate and display readings per second every second
    if current_time - start_time >= 1.0:  # Every 1 second
        print(f"Readings per second: {readings_count}")
        readings_count = 0
        start_time = current_time

    # Display the data
    print(f"Received data: {device.deviceData}")


# Main function
if __name__ == "__main__":
    # Replace "DEVICE_ADDRESS_HERE" with your device's Bluetooth address
    BLE_DEVICE_ADDRESS = ble_address
    device = DeviceModel("WTWitmotion", BLE_DEVICE_ADDRESS, process_data_callback)

    # Start the BLE connection
    try:
        asyncio.run(device.openDevice())
    except KeyboardInterrupt:
        device.closeDevice()
