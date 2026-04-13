import os
import fcntl
import time
import threading
from simple_pid import PID
from sms_sts.brakes import Brakes
I2C_DEV = "/dev/i2c-2"
I2C_SLAVE = 0x0703


#The MCP4725 has support from 2 addresses
BUS_ADDRESS = [0x62,0x63]

#The device supports a few power down modes on startup and during operation 
POWER_DOWN_MODE = {'Off':0, '1k':1, '100k':2, '500k':3}
MAX_DAC=3000
class MCP4725:
    def __init__(self, address=BUS_ADDRESS[0]) :
        self.address=address
        self.fd = os.open(I2C_DEV, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, self.address)
        self._writeBuffer=bytearray(2)
        self.running = False
        self.velocity_val={}
        #self.curr_dac = 700
        self.pid = PID(10.0,2.0,0.0)
        self.pid.output_limits = (1200,1500)
        self.pid.sample_time=0.1
        self.brakes = Brakes()
        self.brakes.start()

    def write(self,value):
        if value < 0:
            value=0
        value=value & 0xFFF
        self._writeBuffer[0]=(value>>8) & 0xFF
        self._writeBuffer[1]=value & 0xFF
        os.write(self.fd, self._writeBuffer)

    def read(self):
        buf=bytearray(5)
        os.write(self.fd, buf)
        #time.sleep(0.001)
        eeprom_write_busy=(buf[0] & 0x80)==0
        power_down=self._powerDownKey((buf[0] >> 1) & 0x03)
        value=((buf[1]<<8) | (buf[2])) >> 4
        eeprom_power_down=self._powerDownKey((buf[3]>>5) & 0x03)
        eeprom_value=((buf[3] & 0x0f)<<8) | buf[4] 
        return (eeprom_write_busy,power_down,value,eeprom_power_down,eeprom_value)

    def config(self,power_down='Off',value=0,eeprom=False):
        buf=bytearray()
        conf=0x40 | (POWER_DOWN_MODE[power_down] << 1)
        if eeprom:
            #store the powerdown and output value in eeprom
            conf=conf | 0x60
        buf.append(conf)
        #check value range
        if value<0:
            value=0
        value=value & 0xFFF
        buf.append(value >> 4)
        buf.append((value & 0x0F)<<4)
        os.write(self.fd, buf)

    def _powerDownKey(self,value):
        for key,item in POWER_DOWN_MODE.items():
            if item == value:
                return key
    def control_loop(self):
        self.write(700)
        time.sleep(1)
        while self.running:
            self.write(700)
            #time.sleep(6)
            curr_velocity=0
            if hasattr(self, 'velocity_val') and self.velocity_val:
                if self.velocity_val["speed"] < 0:
                    self.brakes.update_brakes(abs(self.velocity_val["speed"]))
                    target_velocity=0
                elif self.velocity_val["speed"]>=0:
                    target_velocity = (self.velocity_val["speed"]/100)*25
                    #print("target velcotiy = ", target_velocity)
                    self.pid.setpoint=target_velocity
                    to_increment = self.pid(curr_velocity)
                    self.brakes.update_brakes(0)
                if target_velocity==0:
                    to_increment=0
                print("dac bits = ",to_increment)
                self.write(int(to_increment))
            time.sleep(0.45)

        """print("start loop")
        #print("velocity : ",self.velocity_vals)
        self.write(700)
        time.sleep(6)
        self.write(1500)
        time.sleep(2)
        self.write(2000)
        time.sleep(2)"""
    def update_array(self, velocity_val):
        self.velocity_val = velocity_val
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.control_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        self.write(0)
        time.sleep(1)
        if hasattr(self, 'thread'):
            self.thread.join()
        self.brakes.stop()
