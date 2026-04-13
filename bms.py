import sys
import time
import threading
import serial
from state import vehicle_state

class DalyBMSUSB:
    def __init__(self, port="/dev/ttyCH341USB0", baud=2_000_000):
        self.port = port
        self.baud = baud
        self.running = False
        
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1,
            )
        except serial.SerialException as e:
            print(f"ERROR: {e}")
            sys.exit(1)

        self._initialize_adapter()

    def _calculate_checksum(self, data: list) -> int:
        return sum(data[2:]) & 0xFF

    def _initialize_adapter(self):
        """Sends the 20-byte configuration packet to the USB adapter."""
        cmd = [
            0xAA, 0x55,
            0x12,
            0x05,   # 0x05 = 250 kbps
            0x02,   # 0x02 = Extended frame
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ]
        cmd.append(self._calculate_checksum(cmd))
        
        self.ser.write(bytes(cmd))
        self.ser.flush()
        time.sleep(1.0)          
        self.ser.reset_input_buffer()
        print("BMS CAN Adapter ready.")

    def _send_query(self):
        """Builds and sends the 0x0400FF80 query frame."""
        bms_id = 0x0400FF80
        data = bytes([0x00] * 8)
        
        pkt = bytearray()
        pkt.append(0xAA)
        pkt.append(0xE0 | len(data)) # 0xE0 = Extended Data Frame
        pkt += bms_id.to_bytes(4, "little")
        pkt += data
        pkt.append(0x55)
        
        self.ser.write(bytes(pkt))
        self.ser.flush()

    def _read_frame(self) -> dict | None:
        """Reads and parses a single CAN frame from the serial buffer."""
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b[0] == 0xAA:
                break

        hdr = self.ser.read(1)
        if not hdr:
            return None
        t = hdr[0]

        if (t & 0xC0) != 0xC0:
            return None

        extended = bool(t & 0x20)
        remote   = bool(t & 0x10)
        dlc      = t & 0x0F
        id_len   = 4 if extended else 2

        rest = self.ser.read(id_len + dlc + 1)
        if len(rest) != id_len + dlc + 1:
            return None
        if rest[-1] != 0x55:
            return None

        if extended:
            can_id = int.from_bytes(rest[0:4], "little")
            data   = rest[4 : 4 + dlc]
        else:
            can_id = int.from_bytes(rest[0:2], "little")
            data   = rest[2 : 2 + dlc]

        return {"id": can_id, "remote": remote, "dlc": dlc, "data": bytes(data)}

    def _process_frame(self, frame):
        """Extracts data from the frame and updates the vehicle state."""
        if frame["remote"]: return
        
        fid = frame['id']
        data = frame['data']

        if fid == 0x04028001:
            sum_v_bytes = data[0:2]
            curr_bytes = data[2:4]
            soc_bytes = data[4:6]
            life_bytes = data[6:7]

            soc = int.from_bytes(soc_bytes, byteorder='big')
            sum_v = int.from_bytes(sum_v_bytes, byteorder='big')
            total_curr = int.from_bytes(curr_bytes, byteorder='big')
            life = int.from_bytes(life_bytes, byteorder='big')
            
            vehicle_state['BMS']['battery_soc'] = soc * 0.1
            vehicle_state['BMS']['total_v'] = sum_v * 0.1
            vehicle_state['BMS']['current'] = (total_curr - 30000) * 0.1
            vehicle_state['BMS']['life'] = life

            # print(f"SOC : {soc * 0.1}%")
            # print(f"Sum voltage : {sum_v * 0.1}V")
            # print(f"Total current : {(total_curr-30000) * 0.1}A")
            # print(f"Life : {life}")

            
        elif fid == 0x04038001:
            pow_bytes = data[0:2]
            total_energy_bytes = data[2:4]
            mos_temp_bytes = data[4:5]
            board_temp_bytes = data[5:6]
            heat_temp_bytes = data[6:7]
            heat_curr_bytes = data[7:8]

            power = int.from_bytes(pow_bytes, byteorder='big')
            total_energy = int.from_bytes(total_energy_bytes, byteorder='big')
            mos_temp = int.from_bytes(mos_temp_bytes, byteorder='big')
            board_temp = int.from_bytes(board_temp_bytes, byteorder='big') 
            heat_temp = int.from_bytes(heat_temp_bytes, byteorder='big')
            heat_curr = int.from_bytes(heat_curr_bytes, byteorder='big')

            vehicle_state['BMS']['total_power'] = power * 0.1
            vehicle_state['BMS']['total_energy'] = total_energy * 0.1
            vehicle_state['BMS']['mos_temperature'] = mos_temp - 40
            vehicle_state['BMS']['board_temperature'] = board_temp - 40
            vehicle_state['BMS']['heat_temperature'] = heat_temp - 40
            vehicle_state['BMS']['heat_current'] = heat_curr

            # print(f"Total power : {power * 0.1}W")
            # print(f"Total Energy : {total_energy * 0.1}WH")
            # print(f"MOS temperature : {mos_temp - 40}*C")
            # print(f"Board Temperature : {board_temp - 40}*C")
            # print(f"Heat Temperature : {heat_temp - 40}*C")
            # print(f"Heat current : {heat_curr}A")

        elif fid == 0x04048001:
            max_v_bytes = data[0:2]
            max_v_n_bytes = data[2:3]
            min_v_bytes = data[3:5]
            min_v_n_bytes = data[5:6]
            diff_v_bytes = data[6:8]

            max_v = int.from_bytes(max_v_bytes, byteorder='big')
            max_v_n = int.from_bytes(max_v_n_bytes, byteorder='big')
            min_v = int.from_bytes(min_v_bytes, byteorder='big')
            min_v_n = int.from_bytes(min_v_n_bytes, byteorder='big') 
            diff_v = int.from_bytes(diff_v_bytes, byteorder='big')

            vehicle_state['BMS']['max_cell_voltage'] = max_v / 1000
            vehicle_state['BMS']['max_cell_voltage_position'] = max_v_n
            vehicle_state['BMS']['min_cell_voltage'] = min_v / 1000
            vehicle_state['BMS']['min_cell_voltage_position'] = min_v_n
            vehicle_state['BMS']['diff_voltage'] = diff_v / 1000

            # print(f"Max cell voltage : {max_v / 1000}V")
            # print(f"Max cell voltage position : {max_v_n}")
            # print(f"Min cell voltage : {min_v/ 1000}V")
            # print(f"Min cell voltage position : {min_v_n}")
            # print(f"Diff Voltage : {diff_v / 1000}V")
        
        elif fid == 0x04058001:
            max_t_bytes = data[0:1]
            max_t_n_bytes = data[1:2]
            min_t_bytes = data[2:3]
            min_t_n_bytes = data[3:4]
            diff_t_bytes = data[4:5]

            max_t = int.from_bytes(max_t_bytes, byteorder='big')
            max_t_n = int.from_bytes(max_t_n_bytes, byteorder='big')
            min_t = int.from_bytes(min_t_bytes, byteorder='big')
            min_t_n = int.from_bytes(min_t_n_bytes, byteorder='big') 
            diff_t = int.from_bytes(diff_t_bytes, byteorder='big')

            vehicle_state['BMS']['max_cell_temperature'] = max_t - 40
            vehicle_state['BMS']['max_cell_temperature_position'] = max_t_n
            vehicle_state['BMS']['min_cell_temperature'] = min_t - 40
            vehicle_state['BMS']['min_cell_temperature_position'] = min_t_n
            vehicle_state['BMS']['diff_temperature'] = diff_t

            # print(f"Max cell temperature : {max_t - 40}*C")
            # print(f"Max cell temperature position : {max_t_n}")
            # print(f"Min cell temperature : {min_t - 40}*C")
            # print(f"Min cell Temperature position : {min_t_n}")
            # print(f"Diff Temperature : {diff_t}*C")

        elif fid == 0x04068001:
            chg_mos_bytes = data[0:1]
            dis_mos_bytes = data[1:2]
            pre_mos_bytes = data[2:3]
            heat_mos_bytes = data[3:4]
            fan_mos_bytes = data[4:5]

            chg_mos = int.from_bytes(chg_mos_bytes, byteorder='big')
            dis_mos = int.from_bytes(dis_mos_bytes, byteorder='big')
            pre_mos = int.from_bytes(pre_mos_bytes, byteorder='big')
            heat_mos = int.from_bytes(heat_mos_bytes, byteorder='big') 
            fan_mos = int.from_bytes(fan_mos_bytes, byteorder='big')

            vehicle_state['BMS']['charging_mos_state'] = chg_mos
            vehicle_state['BMS']['discharge_mos_state'] = dis_mos
            vehicle_state['BMS']['precharge_mos_state'] = pre_mos
            vehicle_state['BMS']['heat_mos_state'] = heat_mos
            vehicle_state['BMS']['fan_mos_state'] = fan_mos

            # print(f"Charging MOS state : {chg_mos}")
            # print(f"Discharge MOS state : {dis_mos}")
            # print(f"PreCharge MOS state : {pre_mos}")
            # print(f"Heat MOS state : {heat_mos}")
            # print(f"Fan MOS state : {fan_mos}")
        
        elif fid == 0x04078001:
            bat_bytes = data[0:1]
            chg_detect_bytes = data[1:2]
            load_detect_bytes = data[2:3]

            bat_state = int.from_bytes(bat_bytes, byteorder='big')
            chg_detect = int.from_bytes(chg_detect_bytes, byteorder='big')
            load_detect = int.from_bytes(load_detect_bytes, byteorder='big')

            vehicle_state['BMS']['battery_state'] = bat_state
            vehicle_state['BMS']['charge_detect'] = chg_detect
            vehicle_state['BMS']['load_detect'] = load_detect

            # print(f"Battery state, 0,1,2 : {bat_state}")
            # print(f"Charge Detect, 0,1 : {chg_detect}")
            # print(f"Load Detect, 0,1 : {load_detect}")
        
        elif fid == 0x04088001:
            cell_n_bytes = data[0:1]
            ntc_n_bytes = data[1:2]
            rem_capacity_bytes = data[2:6]
            cycle_time_bytes = data[6:8]

            cell_n = int.from_bytes(cell_n_bytes, byteorder='big')
            ntc_n = int.from_bytes(ntc_n_bytes, byteorder='big')
            rem_capacity = int.from_bytes(rem_capacity_bytes, byteorder='big')
            cycle_time = int.from_bytes(cycle_time_bytes, byteorder='big') 

            vehicle_state['BMS']['cell_number'] = cell_n
            vehicle_state['BMS']['temperature_number'] = ntc_n
            vehicle_state['BMS']['remaining_capacity'] = rem_capacity
            vehicle_state['BMS']['cycle_time'] = cycle_time

            # print(f"Cell number : {cell_n}")
            # print(f"Temperature number : {ntc_n}")
            # print(f"Remaining Capacity : {rem_capacity}mAH")
            # print(f"Cycle Time : {cycle_time}")
        
        elif fid == 0x040B8001:
            rem_chg_bytes = data[0:2]
            rem_charge = int.from_bytes(rem_chg_bytes, byteorder='big')

            vehicle_state['BMS']['remaining_charge_time'] = rem_charge

            # print(f"Remaining Charge Time : {rem_charge} min")

        elif fid == 0x040D8001:
            lim_curr_state_bytes = data[0:1]
            lim_curr_bytes = data[1:3]

            lim_curr_state = int.from_bytes(lim_curr_state_bytes, byteorder='big')
            lim_curr = int.from_bytes(lim_curr_bytes, byteorder='big')

            vehicle_state['BMS']['limiting_current_state'] = lim_curr_state
            vehicle_state['BMS']['limiting_current'] = (lim_curr - 30000) * 0.1

            # print(f"Limiting Current state, 0,1 : {lim_curr_state}")
            # print(f"Limiting Current : {(lim_curr - 30000) * 0.1}A")

    def control_loop(self):
        """The main thread loop: Queries, listens, processes, and sleeps."""
        while self.running:
            # 1. Send the wake-up/query command
            self._send_query()
            
            # 2. Catch all incoming frames from the BMS for ~400ms
            # Because the BMS replies with multiple frames, we stay in a tight
            # loop catching them until the buffer clears or time runs out.
            end_time = time.time() + 0.40 
            while time.time() < end_time:
                if self.ser.in_waiting > 0:
                    frame = self._read_frame()
                    if frame:
                        self._process_frame(frame)
                else:
                    time.sleep(0.01) # Yield CPU if buffer is empty
            
            # 3. Sleep for the remaining ~100ms to achieve exactly 0.5s loop time
            time.sleep(0.10)

    def start(self):
        """Starts the BMS polling thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.control_loop, daemon=True)
            self.thread.start()
            print("BMS Polling Thread Started.")

    def stop(self):
        """Stops the thread and safely closes the port."""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("BMS Polling Thread Stopped.")


# ==========================================
# Example Usage
# ==========================================
if __name__ == "__main__":
    # 1. Initialize the class
    bms = DalyBMSUSB(port="/dev/ttyCH341USB0")
    
    # 2. Start the background loop
    bms.start()
    
    try:
        # 3. Main program does other things while BMS updates in background
        while True:
            time.sleep(1)
            # print(f"Current State: {vehicle_state['BMS']}")
            
    except KeyboardInterrupt:
        pass
    finally:
        bms.stop()
