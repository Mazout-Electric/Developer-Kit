import time
import math
import threading
import gpiod
from gpiod.line import Direction, Value

class HallSensor:
    def __init__(self, chip_number=9, line_offset=8, pulses_per_rev=53, sampling_time=1.0, wheel_radius_m=0.2):
        self.chip_number = chip_number
        self.line_offset = line_offset
        self.pulses_per_rev = pulses_per_rev
        self.sampling_time = sampling_time
        
        self.wheel_radius_m = wheel_radius_m
        self.circumference = 2 * math.pi * self.wheel_radius_m

        self._current_rpm = 0.0
        self._current_speed_kmh = 0.0
        self._running = False
        
        self._lock = threading.Lock()

    def get_rpm(self):
        with self._lock:
            #print("rpm",self._current_rpm)
            return self._current_rpm

    def get_speed(self):
        with self._lock:
            return self._current_speed_kmh

    def _sensor_loop(self):
        chip_path = f"/dev/gpiochip{self.chip_number}"

        try:
            with gpiod.request_lines(
                chip_path,
                consumer="RPM_Sensor",
                config={
                    self.line_offset: gpiod.LineSettings(
                        direction=Direction.INPUT
                    )
                }
            ) as request:
                
                pulse_count = 0
                start_time = time.time()
                
                while self._running:
                    if request.get_value(self.line_offset) == Value.ACTIVE:
                        pulse_count += 1
                        
                        while request.get_value(self.line_offset) == Value.ACTIVE and self._running:
                            time.sleep(0.0001) 
                    
                    current_time = time.time()
                    elapsed = current_time - start_time
                    
                    if elapsed >= self.sampling_time:
                        rpm = (pulse_count * (60.0 / elapsed)) / self.pulses_per_rev
                        
                        speed_mps = (rpm / 60.0) * self.circumference
                        speed_kmh = speed_mps * 3.6 
                        
                        with self._lock:
                            self._current_rpm = round(rpm, 1)
                            self._current_speed_kmh = round(speed_kmh, 1)
                            
                        pulse_count = 0
                        start_time = time.time()
                        
                    time.sleep(0.0001)

        except Exception as e:
            print(f"Hall Sensor Error: {e}")
        finally:
            print("Hall Sensor loop stopped and GPIO released.")

    def start(self):
        self._running = True
        self.thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self._running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)
