#!/usr/bin/env python
#
# *********     ST Servo Position Control (FIXED)     *********
#

import sys
import os

# ===================== KEY INPUT =====================
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
# ====================================================

sys.path.append("..")
from scservo_sdk import *

# ===================== CONFIG =====================
DEVICENAME = '/dev/ttyACM0'        # Change if needed
BAUDRATE   = 1000000
ACC        = 50
# ================================================

# Initialize port & packet handler
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

# Open port
if not portHandler.openPort():
    print("❌ Failed to open port")
    quit()
print("✅ Port opened")

# Set baudrate
if not portHandler.setBaudRate(BAUDRATE):
    print("❌ Failed to set baudrate")
    quit()
print("✅ Baudrate set")

# ===================== USER INPUT =====================
try:
    SCS_ID = int(input("Enter Servo ID (1–253): "))
    if not 1 <= SCS_ID <= 253:
        raise ValueError

    GOAL_POSITION = int(input("Enter Goal Position (0–4095): "))
    if not 0 <= GOAL_POSITION <= 4095:
        raise ValueError

    SPEED = int(input("Enter Speed (1–3000): "))
    if not 1 <= SPEED <= 3000:
        raise ValueError

except ValueError:
    print("❌ Invalid input")
    portHandler.closePort()
    quit()
# ====================================================

# ===================== MOVE SERVO =====================
print(f"\n➡ Moving Servo {SCS_ID} to position {GOAL_POSITION}")

scs_comm_result, scs_error = packetHandler.WritePosEx(
    SCS_ID,
    GOAL_POSITION,
    SPEED,
    ACC
)

if scs_comm_result != COMM_SUCCESS:
    print("❌", packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("❌", packetHandler.getRxPacketError(scs_error))
else:
    print("✅ Command sent successfully")

print("\nPress any key to exit...")
getch()

portHandler.closePort()
