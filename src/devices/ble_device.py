import asyncio
from bleak import BleakClient
import yaml

class DeviceModel:
    def __init__(self, config_path: str, callback_method):
        """Initialize BLE device with configuration from YAML."""
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)['ble_device']
            
        self.deviceName = config['name']
        self.BLEDevice = config['address']
        self.service_uuid = config['service_uuid']
        self.char_uuid_read = config['characteristic_uuid']['read']
        self.char_uuid_write = config['characteristic_uuid']['write']
        self.output_rate = config['output_rate']
        
        self.client = None
        self.writer_characteristic = None
        self.isOpen = False
        self.callback_method = callback_method
        self.deviceData = {}
        self.TempBytes = []

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

            notify_characteristic = None

            print("Discovering services and characteristics...")
            for service in client.services:
                if service.uuid == self.service_uuid:
                    for characteristic in service.characteristics:
                        if characteristic.uuid == self.char_uuid_read:
                            notify_characteristic = characteristic
                        elif characteristic.uuid == self.char_uuid_write:
                            self.writer_characteristic = characteristic
                    if notify_characteristic:
                        break

            if notify_characteristic:
                print("Setting up notifications...")
                await client.start_notify(notify_characteristic.uuid, self.onDataReceived)

                if self.writer_characteristic:
                    print(f"Configuring sensor for {self.output_rate} Hz output...")
                    await self.setOutputRate(self.output_rate)

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
            10: 0x0A, 20: 0x14, 50: 0x32,
            100: 0x64, 200: 0xC8, 500: 0xF4
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