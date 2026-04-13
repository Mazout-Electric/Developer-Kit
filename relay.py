import usb.util
import pyhid_usb_relay
from pyhid_usb_relay.exceptions import DeviceNotFoundError
import time

original_get_string = usb.util.get_string

def safe_get_string(*args, **kwargs):
    try:
        return original_get_string(*args, **kwargs)
    except ValueError:
        return ""

usb.util.get_string = safe_get_string

relay_board = None

def get_relay():
    global relay_board
    
    if relay_board is not None:
        return relay_board
        
    try:
        relay_board = pyhid_usb_relay.find()
        return relay_board
    except DeviceNotFoundError:
        return None
    except Exception as e:
        print(f"Error checking USB relay: {e}")
        return None

def toggle_state(channel):
    global relay_board
    board = get_relay()
    
    if board is None:
        #print(f"USB is disconnected.")
        return

    try:
        #print("relay")
        board.toggle_state(channel)
    except Exception as e:
        #print(f"⚠️ Lost connection to relay board during toggle: {e}")
        relay_board = None 

def set_relay_state(channel, target_state):
    global relay_board
    board = get_relay()
    
    if board is None:
        return

    try:
        current_status = board.get_state(channel)
        
        is_on = bool(current_status)
        
        if is_on != target_state:
            board.toggle_state(channel)
            #print(f"Relay {channel} physically changed to {target_state}")
    except Exception as e:
        #print(f"Lost connection to relay board during state check: {e}")
        relay_board = None

def relay_shutdown():
    board = get_relay()
    if board is not None:
        #print("Relay OFF...")
        for i in range(1, 9):
            set_relay_state(i, False)

def test():
    print("Testing relay channel 2...")
    toggle_state(2)
    time.sleep(2)
    toggle_state(2)
