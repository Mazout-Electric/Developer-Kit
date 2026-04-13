import paho.mqtt.client as mqtt
import ssl
import json
import time
import threading 
import sys

from state import vehicle_state
from vehicle_io import VehicleIO
from remote_control import MCP4725
from remote_steering import ActuatorController
from relay import *
from hall import HallSensor
from bms import DalyBMSUSB
from ups import INA219


bms = DalyBMSUSB(port="/dev/ttyCH341USB0")
bms.start()

def init_peripherals():
    hw = {'hall': None, 'vehicleIO': None, 'dac': None, 'steering': None, 'relay': None, 'bms': None, 'ups': None}
    
    while True:
        try:
            if not hw['bms']:
                hw['bms'] = DalyBMSUSB(port="/dev/ttyCH341USB0")
                hw['bms'].start()
            if not hw['ups']:
                hw['ups'] = INA219()
            if not hw['hall']: 
                hw['hall'] = HallSensor()
                hw['hall'].start()
            if not hw['vehicleIO']: 
                hw['vehicleIO'] = VehicleIO()
                hw['vehicleIO'].start()
            if not hw['dac']: 
                hw['dac'] = MCP4725()
                hw['dac'].start()
            if not hw['steering']: 
                hw['steering'] = ActuatorController(mode="uart", wheelbase_m=1.2, lookahead_index=5)
                hw['steering'].start()
                
            if not hw['relay']:
                hw['relay'] = get_relay()
                if hw['relay'] is None:
                    raise Exception("Relay Board not found!")
            
            return hw['hall'], hw['vehicleIO'], hw['dac'], hw['steering'], hw['ups']
            
        except Exception as e:
            print(f"Hardware Init Error: {e}")
            time.sleep(3)

hall, vehicleIO, dac, steering, ups = init_peripherals()

client_id = "MZ01-ETA-0002"
broker = "13.204.67.195"
broker_port = 8883
subscribe_topic = f"devices/{client_id}/control"
publish_topic = f"devices/{client_id}/telemetry"

last_control_msg_time = time.time()
safe_mode_active = False

def engage_safe_mode(reason):
    global safe_mode_active
    if not safe_mode_active:
        print("Safe mode")
        safe_mode_active = True
        try:
            dac.update_array({"motion": "stop", "speed": 0, "angle": 0})
            steering.ingest_kappa({"motion": "stop", "speed": 0, "angle": 0})
        except Exception as e:
            print(f"Error:  {e}")

def disengage_safe_mode():
    global safe_mode_active
    if safe_mode_active:
        print("Safe Mode Disengaged.")
        safe_mode_active = False

def publish_loop():
    global last_control_msg_time
    
    while True:
        try:
            if time.time() - last_control_msg_time > 5:
                engage_safe_mode("Operator timeout / No recent commands")
            
            telemetry_payload = json.dumps(vehicle_state)
            client.publish(publish_topic, telemetry_payload)
        except Exception as e:
            print(f"Publish error: {e}")
            
        time.sleep(1.0)

def listener_thread():
    while True:
        try:
            vehicle_state['speed'] = hall.get_speed()
            vehicle_state['rpm'] = hall.get_rpm()
            bus_v = ups.getBusVoltage_V()
            current = ups.getCurrent_mA()
            power = ups.getPower_W()
            p = (bus_v - 3) / 1.2 * 100
            p = max(0, min(100, p))
            vehicle_state['UPS']['bus_v'] = round(bus_v,3)
            vehicle_state['UPS']['current'] = round((current/1000),3)
            vehicle_state['UPS']['power'] = round(power,3)
            vehicle_state['UPS']['percentage'] = round(p,1)
            #print("speed",vehicle_state['speed'])
        except Exception as e:
            print(f"Error: {e}")
        time.sleep(0.1)

def on_message(client, userdata, msg):
    global last_control_msg_time
    raw_payload = msg.payload.decode('utf-8')
    
    try:
        json_data = json.loads(raw_payload)
        print("json data : ", json_data)
        last_control_msg_time = time.time() 
        disengage_safe_mode()
        
        if json_data.get("motion"):
            dac.update_array(json_data)
            steering.ingest_kappa(json_data)


        if "horn" in json_data:
            set_relay_state(7, json_data["horn"])
            
        if "killSwitch" in json_data:
            set_relay_state(2, json_data["killSwitch"])
            
        if "backlight" in json_data:
            set_relay_state(8, json_data["backlight"])

        indicator = json_data.get("indicator")
        if indicator == "left":
            toggle_state(5)
        elif indicator == "right":
            toggle_state(6)
        elif indicator == "off":
            set_relay_state(5, False)
            set_relay_state(6,False)
        gear = json_data.get("relay")
        if gear == "forward":
            set_relay_state(1,False)
        elif gear == "reverse":
            set_relay_state(1,True)
    except json.JSONDecodeError:
        print(f"Bad JSON received: {raw_payload}")
    except Exception as e:
        print(f"Error processing message: {e}")

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print(f"Connected to broker at {broker}:{broker_port}!")
        client.subscribe(subscribe_topic)
    else:
        print(f"Connection failed with code: {reason_code}")

def on_disconnect(client, userdata, flags, reason_code, properties):
    print(f"Unexpected disconnection (Error Code: {reason_code})")
    engage_safe_mode("MQTT Disconnected")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect
client.tls_set(cert_reqs=ssl.CERT_NONE)
client.tls_insecure_set(True)

print(f"Connecting to {broker}:{broker_port}...")
client.connect(broker, broker_port, 60)

client.loop_start() 

threading.Thread(target=listener_thread, daemon=True).start()

try:
    publish_loop()
    
except KeyboardInterrupt:
    print("\nShutting down system safely...")
    client.loop_stop()
    client.disconnect()
    try:
        dac.stop()
        relay_shutdown()
        steering.stop()
        hall.stop()
        vehicleIO.stop()
    except Exception as e:
        print(f"Error during shutdown: {e}")
    print("Exited cleanly.")
