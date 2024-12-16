from xarm.wrapper import XArmAPI
import yaml

class ArmController:
    def __init__(self, config_path: str):
        """Initialize the arm controller with configuration from YAML."""
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)['robotic_arm']
        
        self.arm = XArmAPI(config['ip_address'])
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(config['settings']['mode'])
        self.arm.set_state(config['settings']['initial_state'])

    def move_with_angles(self, roll, pitch, yaw):
        """Move the arm to specified orientation angles."""
        roll = roll % 360
        pitch = pitch % 360
        yaw = yaw % 360
        
        code, curr_pos = self.arm.get_position()
        self.arm.set_servo_cartesian(
            [curr_pos[0], curr_pos[1], curr_pos[2], roll, pitch, yaw],
            is_radian=False,
            wait=False
        )