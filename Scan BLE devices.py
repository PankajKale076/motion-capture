import asyncio
from bleak import BleakScanner

async def scan_devices():
    print("Scanning for Bluetooth devices...")
    devices = await BleakScanner.discover()
    for i, device in enumerate(devices):
        print(f"{i + 1}. Name: {device.name}, Address: {device.address}")

if __name__ == "__main__":
    asyncio.run(scan_devices())
