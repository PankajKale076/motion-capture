import asyncio
import numpy as np
from bleak import BleakClient
from xarm.wrapper import XArmAPI

# Configuration: Define BLE device address and xArm IP here
BLE_DEVICE_ADDRESS = "FB:B1:17:87:57:EC"  # Replace with your BLE device address
ARM_IP = "192.168.1.211"  # Replace with your xArm's IP address

# Initialize xArm
arm = XArmAPI(ARM_IP)
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)

# Device Model Class
class DeviceModel:
    def __init__(self, device_name, ble_device, callback_method):
        self.device_name = device_name
        self.ble_device = ble_device
        self.client = None
        self.callback_method = callback_method
        self.device_data = {}
        self.temp_bytes = []
        self.acc_buffer = []
        self.filter_window = 5
        self.gravity_comp = [0, 0, 9.81]  # Gravity compensation placeholder

    async def open_device(self):
        try:
            async with BleakClient(self.ble_device, timeout=15) as client:
                self.client = client

                target_service_uuid = "0000ffe5-0000-1000-8000-00805f9a34fb"
                notify_characteristic_uuid = "0000ffe4-0000-1000-8000-00805f9a34fb"

                print("Setting up notifications...")
                await client.start_notify(notify_characteristic_uuid, self.on_data_received)

                while True:
                    await asyncio.sleep(1)
        except Exception as e:
            print(f"Error connecting to BLE device: {e}")

    def on_data_received(self, sender, data):
        self.temp_bytes.extend(data)
        if len(self.temp_bytes) >= 20:
            self.process_data(self.temp_bytes[:20])
            self.temp_bytes = self.temp_bytes[20:]

    def process_data(self, bytes_data):
        if bytes_data[1] == 0x61:  # Acceleration and angle data
            ax = self.get_signed_int(bytes_data[2:4]) / 32768 * 16 * 9.81
            ay = self.get_signed_int(bytes_data[4:6]) / 32768 * 16 * 9.81
            az = self.get_signed_int(bytes_data[6:8]) / 32768 * 16 * 9.81
            roll = self.get_signed_int(bytes_data[14:16]) / 32768 * 180
            pitch = self.get_signed_int(bytes_data[16:18]) / 32768 * 180
            yaw = self.get_signed_int(bytes_data[18:20]) / 32768 * 180

            smoothed_acc = self.smooth_acceleration([ax, ay, az])
            smoothed_acc[2] -= self.gravity_comp[2]  # Subtract gravity from Z

            # Update device data
            self.device_data = {
                "acc_x": smoothed_acc[0],
                "acc_y": smoothed_acc[1],
                "acc_z": smoothed_acc[2],
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw
            }
            self.callback_method(self.device_data)

    def smooth_acceleration(self, acc_values):
        self.acc_buffer.append(acc_values)
        if len(self.acc_buffer) > self.filter_window:
            self.acc_buffer.pop(0)
        return np.mean(self.acc_buffer, axis=0)

    @staticmethod
    def get_signed_int(data):
        value = int.from_bytes(data, byteorder='little', signed=False)
        return value - 65536 if value >= 32768 else value

# Callback to process IMU data and move the robotic arm
def process_data_callback(device_data):
    acc_x = device_data["acc_x"] * 0.01  # Scale to robot's coordinate system
    acc_y = device_data["acc_y"] * 0.01
    acc_z = device_data["acc_z"] * 0.01

    roll = device_data["roll"]
    pitch = device_data["pitch"]
    yaw = device_data["yaw"]

    # Map IMU data to joint positions
    joint_positions = [
        np.clip(acc_x, -90, 90),  # Joint 1
        np.clip(acc_y, -90, 90),  # Joint 2
        np.clip(acc_z, -90, 90),  # Joint 3
        np.clip(roll, -90, 90),   # Joint 4
        np.clip(pitch, -90, 90),  # Joint 5
        np.clip(yaw, -90, 90)     # Joint 6
    ]

    print(f"Moving Robot to Positions: {joint_positions}")
    arm.set_servo_angle(angle=joint_positions, is_radian=False)

# Main logic
if __name__ == "__main__":
    imu_device = DeviceModel("WTWitmotion", BLE_DEVICE_ADDRESS, process_data_callback)
    try:
        asyncio.run(imu_device.open_device())
    except KeyboardInterrupt:
        print("Operation interrupted.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        arm.disconnect()

