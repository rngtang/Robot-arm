import time
import sys

sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')

from controls import Controls

controls = Controls()
controls.test_controls()

# controls = Controls()
# controls.send_angles([0, 0, 0, 0, 0, 0], speed=70)
# controls.send_coords([249, 32, -12, 180, 1, -48], speed=70, model=2)
# controls.get_angles()
# controls.get_coords()
# controls.toggle_servo_released(True) -> release all servos
# controls.toggle_servo_released(False) -> lock all servos (this function is not well-tested yet)
# controls.test_controls() -> predefined function to test controls functionatity