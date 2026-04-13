import sys
import os
import threading
import time
from .scservo_sdk import *
sys.path.append('..')

from state import vehicle_state

# --- Configuration ---
MOTOR1_ID  = 1
MOTOR2_ID  = 2
DEVICENAME = '/dev/ttySTM4'  # <-- Updated for STM32MP2 UART4
BAUDRATE   = 1000000
SPEED      = 2400
ACC        = 50

class Brakes:
    def __init__(self):
        self.running = False
        self.to_brake = 0.0
        self._last_sent_brake = None # Tracks the last applied brake state
        
        # Lock ensures the main thread and background thread don't crash the serial port
        self.serial_lock = threading.Lock()
        
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = sms_sts(self.portHandler)
        
        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(BAUDRATE):
            print(f"[!] Failed to connect to {DEVICENAME}. Ensure port is correct and Motor is powered.")
            sys.exit(1)

    def move_to_angle(self, m_id, target_angle):
        """Converts degrees to 0-4095 and writes to motor safely."""
        pos_target = int((target_angle / 360.0) * 4095)
        pos_target = max(0, min(4095, pos_target)) 
        
        with self.serial_lock:
            self.packetHandler.WritePosEx(m_id, pos_target, SPEED, ACC)
        time.sleep(0.1) # <--- EXTREME DELAY (100ms)
        return pos_target

    def get_current_angle(self, m_id):
        with self.serial_lock: 
            pos, speed, res, err = self.packetHandler.ReadPosSpeed(m_id)
        time.sleep(0.1) # <--- EXTREME DELAY (100ms)
        
        if res == COMM_SUCCESS:
            angle = round((pos * 360.0) / 4095.0, 1)
            return angle, pos
        return None, None

    def enableT(self, m_id, enable):
        with self.serial_lock: 
            res = self.packetHandler.enableTorque(m_id, enable)
        time.sleep(0.1) # <--- EXTREME DELAY (100ms)
        return res

    def readtemp(self, m_id):
        with self.serial_lock: 
            temp, res, err = self.packetHandler.ReadTemper(m_id)
        time.sleep(0.1) # <--- EXTREME DELAY (100ms)
        
        if res == COMM_SUCCESS:
            return temp
        return None

    def readLoad(self, m_id):
        with self.serial_lock: 
            load, res, err = self.packetHandler.ReadLoad(m_id)
        time.sleep(0.1) # <--- EXTREME DELAY (100ms)
        
        if res == COMM_SUCCESS:
            return load
        return None

    def update_brakes(self, value):
        """Calculates brake angle based on percentage (0-100%)."""
        value = max(0, min(100, value))
        angle = (value / 100.0) * 40.0
        self.to_brake = angle

    def control_loop(self):
        print("Brake control loop started.")
        try:
            while self.running:
                try: 
                    curr_ang, curr_pos = self.get_current_angle(MOTOR1_ID)
                    curr_ang1, curr_pos1 = self.get_current_angle(MOTOR2_ID)
                    
                    target_brake = self.to_brake

                    if target_brake != self._last_sent_brake:
                        if target_brake == 0:
                            print("brakes released")
                            print("to brake = ", target_brake)
                            self.move_to_angle(MOTOR1_ID, 150)
                            self.move_to_angle(MOTOR2_ID, 280)
                            self.enableT(MOTOR1_ID, 0)
                            self.enableT(MOTOR2_ID, 0)
                            
                        else:
                            target_angle = 150 - target_brake
                            print("brakes applied")
                            print("to brake = ", target_angle)
                            self.enableT(MOTOR1_ID, 1)
                            self.enableT(MOTOR2_ID, 1)
                            self.move_to_angle(MOTOR1_ID, 120)
                            self.move_to_angle(MOTOR2_ID, 310)
                            
                            load1 = self.readLoad(MOTOR1_ID)
                            temp1 = self.readtemp(MOTOR1_ID)
                            load2 = self.readLoad(MOTOR2_ID)
                            temp2 = self.readtemp(MOTOR2_ID)
                            
                            print(" LOAD TAKEN BY MOTOR1 LEFT = ", load1)
                            print(" TEMP BY MOTOR1 LEFT = ", temp1)
                            print(" LOAD TAKEN BY MOTOR2 RIGHT = ", load2)
                            print(" TEMP BY MOTOR2 RIGHT = ", temp2)
                            
                            vehicle_state["brakes_temperature_right"] = temp2
                            vehicle_state["brakes_temperature_left"] = temp1
                            vehicle_state["brakes_load_right"] = load2
                            vehicle_state["brakes_load_left"] = load1
                            
                            ang1, _ = self.get_current_angle(MOTOR1_ID)
                            ang2, _ = self.get_current_angle(MOTOR2_ID)
                            vehicle_state["brakes_angle_right"] = ang2
                            vehicle_state["brakes_angle_left"] = ang1
                            
                        self._last_sent_brake = target_brake
                
                except Exception as e:
                    print(f"[!] Serial glitch in brake loop: {e}")
                
                time.sleep(0.05) 
                
        finally:
            print("\nShutting down brakes...")
            try:
                self.move_to_angle(MOTOR1_ID, 150)
                self.move_to_angle(MOTOR2_ID, 280)
                self.enableT(MOTOR1_ID, 0)
                self.enableT(MOTOR2_ID, 0)
                time.sleep(0.1) 
            except Exception as e:
                print(f"Error moving to home position during shutdown: {e}")
                
            with self.serial_lock:
                self.portHandler.closePort()
            print("Port Closed. Thread exiting.")

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.control_loop, daemon=True)
        self.thread.start()
        print("Brake thread initialized.")

    def stop(self):
        print("Stopping brake thread...")
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)
