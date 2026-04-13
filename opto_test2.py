import time
import gpiod
from gpiod.line import Direction, Value, Bias

# --- Configuration Mapping ---
CHIP_CONFIG = {
    6: [3, 4],           # PG3, PG4
    8: [3, 8, 9, 10],    # PI3, PI8, PI9, PI10
    9: [0, 8]            # PZ0, PZ8
}

HEADER_MAP = {
    (9, 0):  "Pin 29 (PZ0)",
    (6, 3):  "Pin 32 (PG3)",
    (8, 3):  "Pin 35 (PI3)",
    (6, 4):  "Pin 36 (PG4)",
    (8, 8):  "Pin 38 (PI8)",
    (8, 9):  "Pin 12 (PI9)",
    (8, 10): "Pin 22 (PI10)",
    (9, 8):  "Pin 26 (PZ8)"
}

requests = {}
previous_states = {} # Added to remember the last state of each pin

print("Initializing GPIO lines for Optocouplers...")

try:
    # 1. Setup Hardware (Logic is unchanged)
    for chip_num, lines in CHIP_CONFIG.items():
        line_settings = {
            line: gpiod.LineSettings(
                direction=Direction.INPUT,
                bias=Bias.PULL_UP,
                active_low=True 
            ) for line in lines
        }
        
        requests[chip_num] = gpiod.request_lines(
            f"/dev/gpiochip{chip_num}",
            consumer="Optocoupler",
            config=line_settings
        )
        
        # Initialize our memory: assume all pins start as INACTIVE (Idle)
        for line in lines:
            previous_states[(chip_num, line)] = Value.INACTIVE

    print("Monitoring pins... Waiting for triggers. Press Ctrl+C to stop.\n")

    # 2. Main Loop
    while True:
        for chip_num, req in requests.items():
            values = req.get_values()
            requested_lines = CHIP_CONFIG[chip_num]
            
            for line, current_val in zip(requested_lines, values):
                # Look up what the pin was doing a fraction of a second ago
                prev_val = previous_states[(chip_num, line)]
                
                # IF it just changed from IDLE to TRIGGERED -> Print it!
                if current_val == Value.ACTIVE and prev_val == Value.INACTIVE:
                    pin_label = HEADER_MAP.get((chip_num, line), f"Unknown Pin")
                    print(f"⚡ {pin_label} was TRIGGERED!")
                
                # Update our memory for the next loop
                previous_states[(chip_num, line)] = current_val
                
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nExiting and releasing GPIO lines...")

finally:
    # 3. Cleanup
    for req in requests.values():
        req.release()
