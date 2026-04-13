
import math
import time
import threading
import serial

class ActuatorController:
    def __init__(self, mode="uart", wheelbase_m=1.3, lookahead_index=5):
        self.mode = mode.lower()
        self.WHEELBASE = wheelbase_m
        self.LOOKAHEAD = lookahead_index
        
        self.serial_conn = None
        self.lock = threading.Lock()
        #self.latest_kappa_array = None
        self.remote_cmd = {}
        self.last_update_time = 0.0
        self.running = False
        self.max_speed_var = 15.0 
        self.HZ = 50
        self.DT = 1.0 / self.HZ
        self.WATCHDOG_TIMEOUT = 6.0

        self._setup_connection()

    def _setup_connection(self):
        try:
            if self.mode == "uart":
                self.serial_conn = serial.Serial(port="/dev/ttyCH341USB0", baudrate=115200, timeout=0.1)
                print("Initializing Actuator...")
                time.sleep(2)
                
                self.serial_conn.write(b"\r\nN0\r\n")
                time.sleep(0.5)
                
                self.serial_conn.write(b"T1\r\n")
                time.sleep(0.5)
                
        except Exception as e:
            print(f"ERROR Hardware setup failed: {e}")

    def ingest_kappa(self, cmd):
        with self.lock:
            #self.latest_kappa_array = kappa_array
            self.remote_cmd = cmd
            #print("set angle")
            self.last_update_time = time.time()

    def _calculate_angle(self, kappa_val):
        # delta = arctan(L * kappa)
        steering_rad = math.atan(self.WHEELBASE * kappa_val)
        steering_deg = math.degrees(steering_rad)
        
        return max(-45.0, min(45.0, steering_deg))

    def _send_angle(self, angle_deg):
        if self.mode == "uart" and self.serial_conn:
            cmd = f"A{angle_deg:.2f}\r\n"
            self.serial_conn.write(cmd.encode('ascii'))

    def _send_stop(self):
        if self.mode == "uart" and self.serial_conn:
            self.serial_conn.write(b"S\r\n")
            print("Watchdog Triggered: STOP Sent.")

    def _control_loop(self):
        self.cmd_set_max_speed()
        while self.running:
            loop_start = time.time()
            
            #with self.lock:
                #k_array = self.latest_kappa_array
                #to_mv = self.angle_to_mv
                #last_recv = self.last_update_time
            #curr_angle=0
            #target_angle=20
            #self._send_angle(target_angle*3)
            if hasattr(self, 'remote_cmd') and self.remote_cmd:
                target_angle = (self.remote_cmd["angle"]/100)*120
                #print("target angle = ", target_angle)
                if target_angle==0:
                    #print("cmd stop")
                    self.cmd_stop()
                    continue
                self._send_angle(target_angle*3)
            #time.sleep(2)
 



            
            if self.serial_conn.in_waiting > 500:
                self.serial_conn.reset_input_buffer()

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, self.DT - elapsed))

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()

    def send_text_command(self, cmd: str):

        cmd = cmd.strip()
        if not cmd:
            return

        try:
            if self.mode == "uart":
                if not self.serial_conn:
                    raise RuntimeError("Serial not connected")
                #print("command : ",cmd)
                self.serial_conn.write((cmd + "\n").encode())
                #self.log(f"SER TX: {cmd}")
            else:
                print()
                #self._send_can_equivalent(cmd)
        except Exception as e:
            print()
            #messagebox.showerror("Send Error", str(e))
            #self.log(f"Send failed: {e}")

    def cmd_set_max_speed(self):
        #print("set max speed: ", self.max_speed_var)
        self.send_text_command(f"M{self.max_speed_var}")
    def cmd_stop(self):
        self.send_text_command("S")


    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        self._send_stop()
        if self.serial_conn:
            self.serial_conn.close()
