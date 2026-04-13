import time
import threading
import gpiod
from gpiod.line import Direction, Value, Bias

from state import vehicle_state

class VehicleIO:
    def __init__(self):
        self.running = False
        self.requests = {}
        self.previous_states = {}

        self.CHIP_CONFIG = {
            6: [3],
            8: [3, 8, 9, 10],
            9: [0]
        }

        self.STATE_MAP = {
            (6, 3): "right_indicator",
            (6, 4): "right_indicator",
            (8, 3): "left_indicator",
            (8, 8): "forward",
            (8, 9): "back_light",
            (8, 10): "kill_switch",
            (9, 0): "horn",
            (9, 8): "dummy_pin"
        }

        self._init_hardware()

    def _init_hardware(self):
        for chip_num, lines in self.CHIP_CONFIG.items():
            line_settings = {
                line: gpiod.LineSettings(
                    direction=Direction.INPUT,
                    bias=Bias.PULL_UP,
                    active_low=True 
                ) for line in lines
            }
            
            self.requests[chip_num] = gpiod.request_lines(
                f"/dev/gpiochip{chip_num}",
                consumer="VehicleIO",
                config=line_settings
            )
            
            for line in lines:
                self.previous_states[(chip_num, line)] = Value.INACTIVE

    def monitoring_loop(self):
        while self.running:
            for chip_num, req in self.requests.items():
                values = req.get_values()
                requested_lines = self.CHIP_CONFIG[chip_num]
                
                for line, current_val in zip(requested_lines, values):
                    prev_val = self.previous_states[(chip_num, line)]
                    
                    if current_val == Value.INACTIVE and prev_val == Value.ACTIVE:
                        state_key = self.STATE_MAP.get((chip_num,line))
                        if state_key and state_key in vehicle_state:
                            current_bool = vehicle_state[state_key]
                            vehicle_state[state_key] = False
                            print("GPIO disabled")
                    if current_val == Value.ACTIVE and prev_val == Value.INACTIVE:
                        
                        state_key = self.STATE_MAP.get((chip_num, line))
                        
                        if state_key and state_key in vehicle_state:
                            current_bool = vehicle_state[state_key]
                            vehicle_state[state_key] = True
                            
                            print(f" GPIO Trigger! '{state_key}' updated to {vehicle_state[state_key]}")
                    
                    self.previous_states[(chip_num, line)] = current_val
                    
            time.sleep(0.05)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.monitoring_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            
        for req in self.requests.values():
            req.release()
