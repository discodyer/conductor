import sys

sys.path.append('../conductor/')

from conductor.ArduCopter import ArduCopter

drone = ArduCopter(connect_str="com10",  name="暗夜之翼")

drone.arm_and_takeoff(100)
