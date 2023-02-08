import subprocess
import rmlib
from rmlib.rmtools import utils, poses
from rmlib import robot

print("Start RTDE3 on the SmartHand")
subprocess.Popen("python3.9 /home/nvidia/rmstudio_cu/rmlib/rmlib/arms/ur_servers/ur5_rtde.py 192.168.0.6 8003 ur5_rtde_config.xml".split(" "))
print("RTDE3 Started")
r = robot.Robot()
print("Created robot")
'''
handWidth = robot.hand.get_finger_width()
import rmlib
print("Starting test")print(f"handWidth: {handWidth}")
firmwareVersion = robot.hand.get_firmware_version()
print(f"firmware version: {firmwareVersion}")


val = robot.hand.release()
print(f"Release result: {val}")
# robot.hand.grip()
'''
print("Done with test")