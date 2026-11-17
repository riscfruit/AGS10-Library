import time
from ags10 import AGS10
# from ags10 import diagnose; diagnose(16, 17, 0)   # uncomment if nothing responds

sensor = AGS10(sda=16, scl=17, i2c_id=0, freq=10000)
print("AGS10 at 0x%02X, firmware v%d" % (sensor.addr, sensor.version()))

print("Pre-heating (up to ~2 minutes)...")
sensor.wait_ready(callback=lambda s: print("  waiting... %d s" % s))

while True:
    tvoc = sensor.tvoc()  # reads are automatically spaced >= 1.5 s apart
    if tvoc is not None:
        print("TVOC: %d ppb   Resistance: %.1f kOhm" % (tvoc, sensor.resistance()))
    time.sleep(2)
