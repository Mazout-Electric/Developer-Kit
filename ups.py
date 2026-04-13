import os
import fcntl
import time
import struct

I2C_DEV = "/dev/i2c-2"
I2C_SLAVE = 0x0703

# INA219 Registers
_REG_CONFIG         = 0x00
_REG_SHUNTVOLTAGE   = 0x01
_REG_BUSVOLTAGE     = 0x02
_REG_POWER          = 0x03
_REG_CURRENT        = 0x04
_REG_CALIBRATION    = 0x05

class BusVoltageRange:
    RANGE_16V = 0x00
    RANGE_32V = 0x01

class Gain:
    DIV_1_40MV  = 0x00
    DIV_2_80MV  = 0x01
    DIV_4_160MV = 0x02
    DIV_8_320MV = 0x03

class ADCResolution:
    ADCRES_12BIT_32S = 0x0D

class Mode:
    SANDBVOLT_CONTINUOUS = 0x07

class INA219:
    def __init__(self, addr=0x43):
        self.addr = addr
        self.fd = os.open(I2C_DEV, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, self.addr)

        self._current_lsb = 0.1524   # mA per bit
        self._power_lsb   = 0.003048 # W per bit
        self._cal_value   = 26868

        self.setup()

    def write_reg(self, reg, value):
        # Big endian 16-bit
        data = bytes([reg, (value >> 8) & 0xFF, value & 0xFF])
        os.write(self.fd, data)

    def read_reg(self, reg):
        os.write(self.fd, bytes([reg]))
        time.sleep(0.001)
        data = os.read(self.fd, 2)
        return (data[0] << 8) | data[1]

    def setup(self):
        # Write calibration
        self.write_reg(_REG_CALIBRATION, self._cal_value)

        config = (
            BusVoltageRange.RANGE_16V << 13 |
            Gain.DIV_2_80MV << 11 |
            ADCResolution.ADCRES_12BIT_32S << 7 |
            ADCResolution.ADCRES_12BIT_32S << 3 |
            Mode.SANDBVOLT_CONTINUOUS
        )

        self.write_reg(_REG_CONFIG, config)

    def getShuntVoltage_mV(self):
        value = self.read_reg(_REG_SHUNTVOLTAGE)
        if value > 32767:
            value -= 65536
        return value * 0.01

    def getBusVoltage_V(self):
        value = self.read_reg(_REG_BUSVOLTAGE)
        return (value >> 3) * 0.004

    def getCurrent_mA(self):
        value = self.read_reg(_REG_CURRENT)
        if value > 32767:
            value -= 65536
        return value * self._current_lsb

    def getPower_W(self):
        value = self.read_reg(_REG_POWER)
        if value > 32767:
            value -= 65536
        return value * self._power_lsb


# ================== TEST ==================

if __name__ == "__main__":
    ina = INA219(addr=0x43)

    print("INA219 running on /dev/i2c-2 (same bus as PN532)")

    try:
        while True:
            v_bus = ina.getBusVoltage_V()
            current = ina.getCurrent_mA()
            power = ina.getPower_W()

            p = (v_bus - 3) / 1.2 * 100
            p = max(0, min(100, p))

            print(f"V: {v_bus:6.3f}V | I: {current/1000:6.3f}A | P: {power:6.3f}W | {p:5.1f}%")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped.")
