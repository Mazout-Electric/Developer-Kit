import time
import gpiod
from gpiod.line import Direction, Value, Bias

# --- Configuration Mapping ---
# Dictionary format: { chip_number: [list_of_lines] }
CHIP_CONFIG = {
    6: [3, 4],           # PG3, PG4
    8: [3, 8, 9, 10],    # PI3, PI8, PI9, PI10
    9: [0, 8]            # PZ0, PZ8
}

# A lookup table to make the print output easy to read
HEADER_MAP = {
    (9, 0):  "Pin 29 (PZ0)  ",
    (6, 3):  "Pin 32 (PG3)  ",
    (8, 3):  "Pin 35 (PI3)  ",
    (6, 4):  "Pin 36 (PG4)  ",
    (8, 8):  "Pin 38 (PI8)  ",
    (8, 9):  "Pin 12 (PI9)  ",
    (8, 10): "Pin 22 (PI10) ",
    (9, 8):  "Pin 26 (PZ8)  "
}

# Dictionary to hold our open chip requests
requests = {}

print("Initializing GPIO lines for Optocouplers...")

try:
    # 1. Open requests for all chips dynamically
    for chip_num, lines in CHIP_CONFIG.items():
        chip_path = f"/dev/gpiochip{chip_num}"
        
        # Configure all lines for this chip
        # active_low=True: Treats 0V as ACTIVE (standard for optocouplers)
        # bias=Bias.PULL_UP: Turns on the internal STM32 pull-up resistors
        line_settings = {
            line: gpiod.LineSettings(
                direction=Direction.INPUT,
                bias=Bias.PULL_UP,
                active_low=True 
            ) for line in lines
        }
        
        # Request the lines and store the request object
        requests[chip_num] = gpiod.request_lines(
            chip_path,
            consumer="Optocoupler",
            config=line_settings
        )

    print("Monitoring all pins... Press Ctrl+C to stop.\n")

    # 2. Main Loop
    while True:
        output_string = ""
        
        # Iterate through each chip we have open
        for chip_num, req in requests.items():
            # get_values() returns a dictionary of {line_offset: Value} for all lines on this chip
            values = req.get_values()
            requested_lines = CHIP_CONFIG[chip_num]
            
            for line, val in zip(requested_lines,values):
                pin_label = HEADER_MAP.get((chip_num, line), f"Unknown")
                
                # val will be Value.ACTIVE if the optocoupler is pulling the pin to GND
                state = "TRIGGERED" if val == Value.ACTIVE else "IDLE     "
                
                output_string += f"{pin_label}: {state} | \n"
                
        # Print the status on a single updating line
        print(output_string, end="\r", flush=True)
        
        # Sample rate (check every 0.1 seconds)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nExiting and releasing GPIO lines...")

finally:
    # 3. Cleanup: Ensure all chips are released back to the kernel
    for req in requests.values():
        req.release()
