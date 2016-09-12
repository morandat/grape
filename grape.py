import smbus
from RPi import GPIO
from time import sleep
import logging


BUSES = [0, 1] 
INTERUPT_PIN = 15

_log = logging.getLogger(__name__)

buses = {}
stacks = {}
display = None


def _reverse_bits_of_byte(b):
    return (((b * 0x0802L & 0x22110L) |(b * 0x8020L & 0x88440L)) * 0x10101L >> 16) & 0xff

class ButtonHandler(object):
    def on_pressed(self, display):
        pass
    def on_left(self, display):
        pass
    def on_right(self, display):
        pass

class ButtonPrinter(ButtonHandler):
    def on_pressed(self, display):
        print("pressed")
    def on_left(self, display):
        print("left")
    def on_right(self, display):
        print("right")

class Stack(object):

    def __init__(self, bus, prefix):
        self._prefix = prefix
        self._bus = bus
        self.devices = {}
        for dev in STACK_DEVICES:
            if dev.probe(bus, prefix):
                self.devices[dev] = dev(bus, prefix)
        for dev in self.devices.itervalues():
            try:
                dev.setup()
            except Exception as e:
                _log.error("Unable to setup %s: %s", dev, e)
    def __getitem__(self, kind):
        return self.devices[kind]

class I2CDevice(object):
    CLASS_ADDRESS = 0x00 # Should be defined in each subclass
    PROBE_QUICK = False

    def __init__(self, bus, prefix):
        self._bus = bus
        self._address = self.make_address(prefix)

    def __str__(self):
        return "{0}[0x{1}]".format(self.__class__.__name__, self._address)

    def setup(self):
        pass

    def read_byte(self):
        return self._bus.read_byte(self._address)
    def write_byte(self, value):
        self._bus.write_byte(self._address, value)
    def read_word(self, register):
        return self._bus.read_word_data(self._address, register)
    def write_word(self, register, value):
        self._bus.write_word_data(self._address, register, value)

    @classmethod
    def make_address(cls, prefix):
        return cls.CLASS_ADDRESS + prefix

    @classmethod
    def probe(cls, bus, prefix):
        address = cls.make_address(prefix)
        try:
            if cls.PROBE_QUICK:
                bus.write_quick(address)
            else:
                bus.read_byte(address)
            return True
        except:
            return False
        
class PowerSwitch(I2CDevice):
    CLASS_ADDRESS = 0x38
    L2P = [1, 6, 3, 2, 5, 4]

    def setup(self):
        self.update()
    def update(self):
        self._pin = self.read_byte()
    def write(self):
        self.write_byte(self._pin)
    def status(self):
        s = []
        for i in range(len(self.L2P)):
            if self[i]:
                s.append(i)
        return s
    def __getitem__(self, idx):
        return self._pin & (1 << self.L2P[idx]) == 0
    def __setitem__(self, idx, state):
        if state:
            self._pin &= ~(1 << self.L2P[idx])
        else:
            self._pin |= (1 << self.L2P[idx])
        self.write()
    def start_all(self):
        for idx in range(len(self.L2P)):
            self._pin &= ~(1 << self.L2P[idx])
        self.write()
    def stop_all(self):
        for idx in range(len(self.L2P)):
            self._pin |= 1 << self.L2P[idx]
        self.write()
        

class Temperature(I2CDevice):
    CLASS_ADDRESS = 0x40
    THM_REG = 0x00

    def get(self):
        data = self.read_word(self.THM_REG)
        if data & 0x8000:
            return - (~((data >> 8) & 0x7F)) - (0.5 if data & 0x80 else 0)
        else:
            return (data >> 8) + (0.5 if data & 0x80 else 0)

class PowerMeter(I2CDevice):
    CLASS_ADDRESS = 0x48

    CALIBRATION_VALUE = 0x0666
    CURRENT_DIVIDER = (1000 * 50)

    CONF_REG = 0x00
    CALIB_REG = 0x05
    SHUNT_REG = 0x01
    VOLTAGE_REG = 0x02
    POWER_REG = 0x03
    CURRENT_REG = 0x04

    SHUNT_BUS_CONTINUOUS = 0x0007
    SADCRES_12B_1S = 0x0018
    BADCRES_12B = 0x0400
    GAIN_320MV = 0x1800
    GAIN2_80MV = 0x0800
    BUS_VOLTAGE_RANGE_32V = 0x2000

    def setup(self):
        print("{0:x}".format(self.config_value()))
        self.write_word(self.CONF_REG, self.config_value())
        self.write_word(self.CALIB_REG, self.CALIBRATION_VALUE)

    def config_value(self):
        return self.SHUNT_BUS_CONTINUOUS \
                | self.SADCRES_12B_1S \
                | self.BADCRES_12B \
                | self.GAIN2_80MV \
                | self.BUS_VOLTAGE_RANGE_32V

    def current(self): # in mA
        return self.read_word(self.CURRENT_REG) / self.CURRENT_DIVIDER
    def voltage(self): # in mV
        return (self.read_word(self.VOLTAGE_REG) >> 3) * 4
    def power(self): # in mW
        return self.read_word(self.POWER_REG) / 2
    def shunt(self):
        return self.read_word(self.SHUNT_REG)

class Display(I2CDevice):
    CLASS_ADDRESS = 0x20
    DEFAULT_PREFIX = 0x00
    PROBE_QUICK = True

    RED = 0x08
    GREEN = 0x10

    DELAY = 0.004
    SHORT_DELAY = 0.001

    BTN_PRESS = 0x20
    BTN_LEFT_RIGHT = 0x40 | 0x80
    BUTTONS = BTN_PRESS | BTN_LEFT_RIGHT
    _LEFT_SUCCESSOR = [1, 3, 0, 2]

    def setup(self):
        # PCA init
        self.write_word(0x04, 0x00)
        self.write_word(0x05, 0x00)
        self.write_word(0x06, 0x00)
        self.write_word(0x07, 0xE0)

        # LCD init
        sleep(0.015)
        self._write_state(self._read_state() & 0xF8)
        self._lcd(0x30)
        self._lcd(0x30)
        self._lcd(0x30)
        self._lcd(0x38)
        self._lcd(0x0E)
        self.clear_screen()
        self._lcd(0x06)

        # Interupt
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(INTERUPT_PIN, GPIO.IN)
        #GPIO.add_event_detect(INTERUPT_PIN, GPIO.RISING, callback = self.interrupt)

        self.btn_handler = ButtonHandler()
        self._status = self.btn_status()

    def led(self, index, state):
        state = self._read_state()
        if state:
            self._write_state(state &~ index)
        else:
            self._write_state(state | index)

    def cursor_home(self):
        self._lcd(0x02)

    def clear_screen(self):
        self._lcd(0x01)

    def display_shift(self):
        self._lcd(0x1F)

    def lcd_status(self):
        return self.read_word(0x01) & 0x1C

    def btn_status(self):
        return self.read_word(0x01) & self.BUTTONS

    def print_str(self, msg):
        for c in msg:
            self.print_char(ord(c))

    def print_char(self, char):
        state = self._read_state()
        self._write_state(state | 0x04)
        self.write_word(0x02, _reverse_bits_of_byte(char))
        self._lcd_enable()
        self._write_state(state &~ 0x04)

    def interrupt(self, channel):
        p_state = self._status
        state = self._status = self.btn_status()
        if self.btn_handler is None: return
        if state & self.BTN_PRESS: self.btn_handler.on_pressed(self)
        state = (state & self.BTN_LEFT_RIGHT) >> 6 # FIXME function or constant for this 
        p_state = (p_state & self.BTN_LEFT_RIGHT) >> 6
        if state != p_state:
            if self._LEFT_SUCCESSOR[p_state] == state:
                self.btn_handler.on_left(self)
            else:
                self.btn_handler.on_right(self)

    def _lcd(self, instr):
        self.write_word(0x02, _reverse_bits_of_byte(instr))
        sleep(self.DELAY)
        self._lcd_enable()
        sleep(self.DELAY)
    def _lcd_enable(self):
        state = self._read_state()
        self._write_state(state | 0x01)
        sleep(self.SHORT_DELAY)
        self._write_state(state & ~0x01)
    def _reset(self):
        self.write_word(0x03, 0x03)
    def _read_state(self):
        return self.read_word(0x03)
    def _write_state(self, value):
        self.write_word(0x03, value)
    def _config_rw(self):
        self.write_word(0x03, 0x02)

STACK_DEVICES = [PowerSwitch, Temperature, PowerMeter]

for bus_id in BUSES:
    try:
        _log.debug("Probing SMBus %d.", bus_id)
        bus = smbus.SMBus(bus_id)
        buses[bus_id] = bus
        _log.info("SMBus %d detected.", bus_id)
    except:
        _log.warn("SMBus %d not enabled.", bus_id)

for bid, bus in buses.items():
    _log.debug("Probing display on bus %d.", bid)
    if Display.probe(bus, Display.DEFAULT_PREFIX):
        display = Display(bus, Display.DEFAULT_PREFIX)
        display.setup()
        _log.info("Display found on bus %d.", bid)
    else:
        _log.debug("Display on bus %d not found.", bid)
    for sid in range(8):
        try:
            _log.debug("Probing stack %d on bus %d.", sid, bid)
            bus.read_byte(PowerSwitch.make_address(sid))
            stacks[sid] = Stack(bus, sid)
            _log.info("stack %d on bus %d detected.", sid, bid)
        except Exception as e:
            _log.debug("stack %d on bus %d not found.", sid, bid)
