"""
ags10.py - MicroPython library for AGS10 air quality sensor.
Intended for use with Riscfruit Pico2 Expansion Card to control AGS10 sensor module.

Author: Tan Siret Akinci, with help from generative AI tools

Licensed under GPL-3.0

Copyright (c) 2026, Riscfruit Laboratory
All rights reserved.
"""
import time
from machine import I2C, Pin

__version__ = "1.0.0"

DEFAULT_ADDR = 0x1A
MAX_FREQ = 15000

# Commands
_CMD_TVOC = 0x00
_CMD_RESISTANCE = 0x10
_CMD_VERSION = 0x11


class AGS10Error(OSError):
    # Raised when the sensor cannot be reached even after all recovery steps.
    pass


class AGS10CRCError(OSError):
    # Reply received but checksum wrong. Subclass of OSError so it is retried.
    pass


def _crc8(data, n):
    crc = 0xFF
    for i in range(n):
        crc ^= data[i]
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class AGS10:
    def __init__(self, i2c=None, sda=None, scl=None, i2c_id=1, freq=10000,
                 addr=None, retries=5, timeout_us=200000, min_interval_ms=1500):
        # i2c channel, sda/scl pin numbers = None auto detects
        # addr=None auto-detects the fixed address 0x1A.
        self._sda = sda
        self._scl = scl
        self._id = i2c_id
        self._freq = min(freq, MAX_FREQ)  # the chip does not work above ~15 kHz
        self._timeout = timeout_us
        self._retries = max(1, retries)
        self._min_ms = min_interval_ms
        self._last = None   # ticks_ms of the last TVOC read
        self._rs = None     # None = unknown, True = repeated START works, False = use STOP
        self._can_recover = sda is not None and scl is not None
        self.i2c = i2c
        self.addr = addr

        if self.i2c is None:
            if not self._can_recover:
                raise ValueError("Error: sda and scl pins are required when no i2c object is given")
            self._bus_recover()  # also clears any junk left by a previous crashed run
        if self.addr is None:
            self.addr = self._detect()

    # Bus handling
    def _open_i2c(self):
        if self.i2c is not None:
            try:
                self.i2c.deinit()
            except Exception:
                pass
        try:
            self.i2c = I2C(self._id, sda=Pin(self._sda), scl=Pin(self._scl),
                           freq=self._freq, timeout=self._timeout)
        except TypeError:  # port without a timeout argument
            self.i2c = I2C(self._id, sda=Pin(self._sda), scl=Pin(self._scl),
                           freq=self._freq)

    def _bus_recover(self):
        # Free a bus held by a half-finished transfer, then re-create the I2C object.
        if not self._can_recover:
            return
        if self.i2c is not None:
            try:
                self.i2c.deinit()
            except Exception:
                pass
        scl = Pin(self._scl, Pin.OPEN_DRAIN, Pin.PULL_UP, value=1)
        sda = Pin(self._sda, Pin.OPEN_DRAIN, Pin.PULL_UP, value=1)
        time.sleep_us(50)
        # Clock out up to 9 pulses until the slave releases SDA (slow, ~12 kHz)
        for _ in range(9):
            if sda.value():
                break
            scl.value(0)
            time.sleep_us(40)
            scl.value(1)
            time.sleep_us(40)
        # Manual STOP condition: SDA low -> high while SCL high
        sda.value(0)
        time.sleep_us(40)
        scl.value(1)
        time.sleep_us(40)
        sda.value(1)
        time.sleep_us(50)
        self._open_i2c()

    def _recover(self, attempt):
        # Escalating recovery used between retries. The AGS10 has no soft reset
        # command, so recovery = back off, then free/re-create the bus.
        time.sleep_ms(min(10 << attempt, 400))
        if attempt >= 1:
            self._bus_recover()

    def _retry(self, fn):
        last = None
        for attempt in range(self._retries):
            try:
                return fn()
            except OSError as e:
                last = e
                self._recover(attempt)
        raise AGS10Error(
            "Error: Sensor not responding after %d tries (last error: %r)." % (self._retries, last))

    def _detect(self):
        found = []

        def go():
            found[:] = self.i2c.scan()
            if DEFAULT_ADDR in found:
                return DEFAULT_ADDR
            # Some AGS10 units ignore the address-only probe used by scan(),
            # so also try a real "read version" transaction before giving up.
            try:
                self.addr = DEFAULT_ADDR
                d = self._read_cmd(_CMD_VERSION, 5)
                if _crc8(d, 4) == d[4]:
                    return DEFAULT_ADDR
            except OSError:
                pass
            raise OSError(19)  # ENODEV -> triggers recovery/retry

        try:
            return self._retry(go)
        except AGS10Error as e:
            raise AGS10Error("%s Devices seen on bus: %s (expected 0x1A). "
                             "Check wiring, power and pull-ups."
                             % (e.args[0], [hex(a) for a in found] or "none"))

    # Raw transactions (single attempt, no recovery)
    def _read_cmd_once(self, cmd, n):
        # Datasheet style (STOP between write and read) is tried first.
        order = (False, True) if (self._rs is None or not self._rs) else (True, False)
        last = None
        for style in order:
            try:
                if style:
                    self.i2c.writeto(self.addr, bytes([cmd]), False)  # repeated START
                else:
                    self.i2c.writeto(self.addr, bytes([cmd]))         # STOP, then START
                time.sleep_ms(3)
                data = self.i2c.readfrom(self.addr, n)
                self._rs = style
                return data
            except OSError as e:
                last = e
        raise last

    def _read_cmd(self, cmd, n, polls=4):
        # Write a read-command and fetch n bytes; polls briefly in case the chip is busy.
        last = None
        for _ in range(polls):
            try:
                return self._read_cmd_once(cmd, n)
            except OSError as e:
                last = e
                time.sleep_ms(20)
        raise last

    def _read_block(self, cmd, throttle=False):
        # One checked 5-byte reply (4 data bytes + CRC). Retried as a whole.
        def go():
            if throttle and self._last is not None:
                wait = self._min_ms - time.ticks_diff(time.ticks_ms(), self._last)
                if wait > 0:
                    time.sleep_ms(wait)
            if throttle:
                self._last = time.ticks_ms()
            d = self._read_cmd(cmd, 5)
            if _crc8(d, 4) != d[4]:
                raise AGS10CRCError(74)  # EBADMSG
            return d
        return self._retry(go)

    # Measurements
    def read_raw(self):
        # Returns (ready, tvoc_ppb). tvoc_ppb is only meaningful when ready is True.
        d = self._read_block(_CMD_TVOC, throttle=True)
        ready = (d[0] & 0x01) == 0
        tvoc = (d[1] << 16) | (d[2] << 8) | d[3]
        return ready, tvoc

    @property
    def ready(self):
        # True once the pre-heat is over and readings are valid.
        return self.read_raw()[0]

    def tvoc(self):
        # TVOC in ppb, or None while the sensor is still pre-heating.
        ready, tvoc = self.read_raw()
        return tvoc if ready else None

    def wait_ready(self, timeout_s=180, callback=None):
        # Block until pre-heat is finished. callback(seconds_waited) is called each poll.
        # Returns True if ready, False on timeout.
        t0 = time.ticks_ms()
        while True:
            if self.read_raw()[0]:
                return True
            waited = time.ticks_diff(time.ticks_ms(), t0) // 1000
            if callback:
                callback(waited)
            if waited >= timeout_s:
                return False

    def resistance(self):
        # Sensing element resistance in kOhm (raw unit is 0.1 kOhm).
        # Reply layout matches the TVOC reply: byte 0 = status, bytes 1..3 = 24-bit value.
        # (Reading all 4 bytes as a number gave a constant 0x10 top byte = ~26 GOhm.)
        d = self._read_block(_CMD_RESISTANCE)
        return ((d[1] << 16) | (d[2] << 8) | d[3]) / 10.0

    def version(self):
        # Firmware version byte.
        return self._read_block(_CMD_VERSION)[3]

    def deinit(self):
        try:
            self.i2c.deinit()
        except Exception:
            pass


def diagnose(sda=16, scl=17, i2c_id=0, freq=10000):
    # Print a quick health report for the bus. Run this when nothing works.
    s = Pin(sda, Pin.IN, Pin.PULL_UP)
    c = Pin(scl, Pin.IN, Pin.PULL_UP)
    time.sleep_ms(5)
    print("Idle levels  SDA=%d SCL=%d  (both should be 1; 0 = stuck/shorted line)"
          % (s.value(), c.value()))
    for f in (5000, 10000, 15000):
        try:
            bus = I2C(i2c_id, sda=Pin(sda), scl=Pin(scl), freq=f)
            print("scan @ %5d Hz: %s" % (f, [hex(a) for a in bus.scan()] or "nothing"))
            bus.deinit()
        except OSError as e:
            print("scan @ %5d Hz failed: %r" % (f, e))
    try:
        dev = AGS10(sda=sda, scl=scl, i2c_id=i2c_id, freq=freq, retries=2)
    except OSError as e:
        print("Error: No AGS10 sensor found, ", e)
        return
    print("Found AGS10 sensor at 0x%02X" % dev.addr)
    try:
        print("Firmware version: %d" % dev.version())
        print("Resistance: %.1f kOhm" % dev.resistance())
        ready, tvoc = dev.read_raw()
        print("TVOC: %d ppb (%s)  read style: %s"
              % (tvoc, "ready" if ready else "still pre-heating, value not valid",
                 "repeated START" if dev._rs else "STOP+START"))
    except OSError as e:
        print("Error: Sensor is on the bus but measurement failed, ", e)
