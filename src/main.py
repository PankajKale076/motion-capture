import asyncio
import os
from devices.robot_arm import ArmController
from devices.ble_device import DeviceModel
from utils.data_processing import create_data_callback

CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'config.yaml')

async def main():
    # Initialize the arm controller
    arm_controller = ArmController(CONFIG_PATH)
    
    # Create the data processing callback
    callback = create_data_callback(arm_controller)
    
    # Initialize and start the BLE device
    device = DeviceModel(CONFIG_PATH, callback)
    
    try:
        await device.openDevice()
    except KeyboardInterrupt:
        device.closeDevice()

if __name__ == "__main__":
    asyncio.run(main())