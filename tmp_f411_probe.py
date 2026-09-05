#!/usr/bin/env python3
import time, serial
from pathlib import Path
import sys
sys.path.insert(0, '/home/otomasi/ros/hoverboard-firmware-hack-FOC')
from tools.pio_vesc_upload import frame
p=serial.Serial('/dev/ttyACM0',115200,timeout=0.05,write_timeout=1)
def drain(sec=0.5):
    end=time.monotonic()+sec; out=bytearray()
    while time.monotonic()<end:
        out += p.read(4096)
    return bytes(out)
def line(s):
    p.write((s+'\n').encode()); p.flush()
def tx(pkt):
    line('VESC:TX:M:'+pkt.hex().upper()); return drain(1.0)
p.reset_input_buffer(); p.reset_output_buffer()
line('VESC:MODE:MAINTENANCE'); print('MODE',drain(0.5).decode(errors='replace')[-1000:])
print('FW',tx(frame(bytes([0]))).decode(errors='replace')[-2000:])
size=Path('/home/otomasi/ros/hoverboard-firmware-hack-FOC/.pio/build/APP_F411/firmware.bin').stat().st_size
print('ERASE_SIZE',size)
print('ERASE',tx(frame(bytes([2])+size.to_bytes(4,'big'))).decode(errors='replace')[-2000:])
line('VESC:STATUS'); print('STATUS',drain(0.5).decode(errors='replace')[-1000:])
p.close()
