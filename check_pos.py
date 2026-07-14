import yaml
from motion.motion_controller import get_motion_controller

with open("config.yaml") as f:
    config = yaml.safe_load(f)

motion = get_motion_controller(config)
print("Position (no home()/resume() called):", motion.get_position())