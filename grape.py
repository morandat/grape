import smbus
from RPi import GPIO
from time import sleep
import logging


BUSES = [0, 1] 
INTERUPT_PIN = 7

_log = logging.getLogger(__name__)

buses = {}
stacks = {}


def _reverse_bits_of_byte(b):
    return (((b * 0x0802L & 0x22110L) |(b * 0x8020L & 0x88440L)) * 0x10101L >> 16) & 0xff

class Stack(object):

    def init(self, prefix):
        self._prefix = prefix
        self.devices = {}
        for dev in STACK_DEVICES:
            if dev.probe(prefix):
                self.devices[dev] = dev(bus, prefix)
        for dev in devices:
            try:
                dev.setup()
            except Exception as e:
                _log.error("Unable to setup %s: %s", dev, e)

class I2CDevice(object):
    CLASS_ADDRESS = 0x00 # Should be defined in each subclass

    def __init__(self, bus, prefix):
        _bus = bus
        _address = cls.make_address(prefix)

    def setup(self):
        pass

    def read_byte(self):
        self.bus.read_byte(self._address)
    def write_byte(self, value):
        self.bus.read_byte(self._address, value)
    def read_word(self, register):
        self.bus.read_byte(self._address, register)
    def write_word(self, register, value):
        self.bus.read_byte(self._address, register, value)

    @classmethod
    def make_address(cls, prefix):
        return cls.CLASS_ADDRESS + prefix

    @classmethod
    def probe(cls, bus, prefix):
        address = cls.make_address(prefix)
        try:
            _log.debug("Probing %s at 0x%x.", cls, address)
            bus.read_byte(address)
            _log.debug("Found %s at 0x%x.", cls, address)
            return True
        except:
            _log.debug("%s not found at 0x%x.", cls, address)
            return False
        
class PowerSwitch(I2CDevice):
    CLASS_ADDRESS = 0x38
    L2P = [1, 2, 3, 4, 5, 6]

    def setup(self):
        self.update()
    def update(self):
        self._pin = self.read_byte()
    def write(self):
        self.write_byte(self._pin)
    def status(self):
        s = []
        for i in range(len(L2P)):
            if self[i]:
                s.append(i)
    def __getitem__(self, idx):
        return self._pin & (1 << L2P[idx]) != 0
    def __setitem__(self, idx, state):
        if state:
            self._pin |= (1 << L2P[idx])
        else:
            self._pin &= ~(1 << L2P[idx])
        self.update()
        

class Temperature(I2CDevice):
    CLASS_ADDRESS = 0x40
    THM_REG = 0x00

    def temperature(self):
        data = self.read_word(self, self.THM_REG)
        if data & 0x8000:
            return - (~(data >> 8)) - (0.5 if data & 0x80 else 0)
        else:
            return (data >> 8) + (0.5 if data & 0x80 else 0)

class PowerMeter(I2CDevice):
    CLASS_ADDRESS = 0x48

class Display(I2CDevice):
    RED = 0x01
    GREEN = 0x02

    EXP_REG = 0x03

    DELAY=0.004
    SHORT_DELAY=0.001

    def setup(self):
        #PCA init
        self._dev_write(0x04, [0x00])
        self._dev_write(0x05, [0x00])
        self._dev_write(0x06, [0x00])
        self._dev_write(0x07, [0x1C])
        #LCD init
        sleep(0.015)
        self.__dev_write(0x03, self._dev_read(0x03) & 0x1F)
        self._cmd(0x30)
        self._cmd(0x30)
        self._cmd(0x30)
        self._cmd(0x38)
        self._cmd(0x0C)
        self._cmd(0x01)
        self._cmd(0x07)

    def led(self, index, state):
        status = self._read()
        if state:
            self.write_word(0x03, status | 1 << index)
        else:
            self.write_word(0x03, status & ~(1 << index))

    def cursor_home(self):
        self._cmd(0x02)

    def clear(self):
        self._cmd(0x01)

    def display_shift(self):
        self._cmd(0x1F)

    def print_char(self, msg):
        for c in msg:
            self.print_char(ord(c))

    def print_char(self, char):
        self.write_word(0x03, status | 0x80)
        self.write_word(0x02, _reverse_bits_of_byte(instr))
        self.write_word(0x03, status & ~0x80)

    def _cmd(self, instr):
        self.write_word(0x02, _reverse_bits_of_byte(instr))
        sleep(DELAY)
        __lcd_enable()
        sleep(DELAY)
    def __lcd_enable(self):
        self.write_word(0x03, status | 0x20)
        sleep(SHORT_DELAY)
        self.write_word(0x03, status & ~0x20)
    def _read(self):
        return self.read_word(EXP_REG)
    def _write(self, value):
        self.write_word(EXP_REG, value)
    def _status(self):
        return self._dev_read(0x01) & 0x1C


STACK_DEVICES = [PowerSwitch, Temperature, PowerMeter]

for bus_id in BUSES:
    try:
        _log.info("Probing SMBus %d.", bus_id)
        bus = smbus.SMBus(bus_id)
        buses[bus_id] = bus
        _log.info("SMBus %d detected.", bus_id)
    except:
        _log.warn("SMBus %d not enabled.", bus_id)

for bid, bus in buses.items():
    for sid in range(8):
        try:
            _log.info("Probing stack %d on bus %d.", sid, bid)
            bus.read_byte(PowerSwitch.make_address(sid))
            stacks[sid] = Stack(bus, sid)
            _log.info("stack %d on bus %d detected.", sid, bid)
        except Exception as e:
            print(e)
            _log.info("stack %d on bus %d not found.", sid, bid)
