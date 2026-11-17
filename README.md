# AGS10-Library
Micropython library for AGS10 air quality sensor. Written to be used on [Riscfruit Pico2 Expansion Card](https://github.com/riscfruit/Pico2-Expansion-Card) to control [AGS10 Sensor Module](https://github.com/riscfruit/AGS10-I2C-Sensor).

### Installation
Install via pip inside your project directory:
```bash
pip install ags10-library
```

You can easily upload the library to your RP2040/RP2350 based embedded device (Riscfruit Pico2 Expansion Card, Raspberry Pi Pico, etc) by creating a lib directory inside and copying it there.

### Usage
Import the library and initialize sensor and read all values:

```python
sensor = AGS10(sda=16, scl=17, i2c_id=0, freq=10000)
print("AGS10 at 0x%02X, firmware v%d" % (sensor.addr, sensor.version()))

print("Pre-heating (up to ~2 minutes)...")
sensor.wait_ready(callback=lambda s: print("  waiting... %d s" % s))

while True:
    tvoc = sensor.tvoc()  # reads are automatically spaced >= 1.5 s apart
    if tvoc is not None:
        print("TVOC: %d ppb   Resistance: %.1f kOhm" % (tvoc, sensor.resistance()))
    time.sleep(2)
```

For diagnosis, run the following:
```python
from ags10 import diagnose; diagnose(16, 17, 0)
```

See [ags10-sensor-reading.py](examples/ags10-sensor-reading.py) for loop sensor reading example.

### Licensing
Copyright (c) 2026, Riscfruit Laboratory. This software is developed and distributed with [GPL 3.0](LICENSE).
