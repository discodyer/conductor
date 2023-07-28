import sys

sys.path.append('../conductor/')

from conductor.ArduCopter import ArduCopter

import time

drone = ArduCopter(connect_str="com10",  name="暗夜之翼")

drone.arm_and_takeoff(100)

time.sleep(3)

drone.send_ned_velocity(0.1, 0, 0)

time.sleep(3)

drone.land()

drone.close()
