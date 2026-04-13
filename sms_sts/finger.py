#!/usr/bin/env python

import sys
import os
import time

sys.path.append("..")
from scservo_sdk import *

# ===================== CONFIG =====================
DEVICENAME = 'COM5'
BAUDRATE   = 1000000
ACC        = 30           # lower = smoother
SPEED      = 800          # keep low for fingers
STEP_DELAY = 0.02         # 20 ms between steps
STEPS      = 40           # more steps = smoother
# ================================================

# Finger motor IDs
RING  = [4, 5, 6]          # abd, pip, mcp
PINKY = [11, 10, 13]       # abd, pip, mcp

# Neutral (open hand) positions – adjust to your robot
OPEN_POS = {
    4: 2048, 5: 2048, 6: 2048,
    11: 2048, 10: 2048, 13: 2048
}

# Closed finger positions – adjust carefully!
CLOSE_POS = {
    4: 1900, 5: 2600, 6: 2600,
    11: 1900, 10: 2600, 13: 2600
}

# ==================================================

portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort():
    print("❌ Failed to open port")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("❌ Failed to set baudrate")
    quit()

print("✅ Servo system ready")

# ===================== SMOOTH MOVE FUNCTION =====================
def smooth_move(start_pos, end_pos):
    for step in range(STEPS + 1):
        alpha = step / STEPS
        for motor_id in end_pos:
            pos = int(
                start_pos[motor_id] +
                alpha * (end_pos[motor_id] - start_pos[motor_id])
            )
            packetHandler.WritePosEx(
                motor_id,
                pos,
                SPEED,
                ACC
            )
        time.sleep(STEP_DELAY)
# ===============================================================

print("\nPress:")
print("  c → Close ring + pinky")
print("  o → Open ring + pinky")
print("  q → Quit")

while True:
    key = input("\nCommand: ").lower()

    if key == 'c':
        print("✊ Closing fingers")
        smooth_move(OPEN_POS, CLOSE_POS)

    elif key == 'o':
        print("🖐 Opening fingers")
        smooth_move(CLOSE_POS, OPEN_POS)

    elif key == 'q':
        break

portHandler.closePort()
print("👋 Done")
