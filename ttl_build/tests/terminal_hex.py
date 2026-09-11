"""COM3 monitor for the binary protocol; transmit HEX and print replies as HEX."""
import argparse, serial, time

ap=argparse.ArgumentParser()
ap.add_argument('packet', help='packet bytes, for example: FF FF 08 01 01 01 00 55')
ap.add_argument('--port', default='COM3')
ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--bytes', type=int, default=32)
ap.add_argument('--timeout', type=float, default=1.0)
ap.add_argument('--recrc', action='store_true', help='replace final byte with protocol CRC8')
a=ap.parse_args()
raw=bytes.fromhex(a.packet)
given=raw[-1] if raw else 0
crc=0
for byte in raw[:-1]:
    cur=byte
    for _ in range(8):
        bit=(crc>>7) ^ (cur&1)
        crc=((crc<<1)^0x07 if bit else (crc<<1)) & 0xff
        cur >>= 1
if raw and given != crc:
    print('WARNING: CRC mismatch; packet has %02X, expected %02X (use --recrc)' % (given,crc))
if a.recrc:
    raw=raw[:-1]+bytes([crc])
p=serial.Serial(a.port,a.baud,timeout=.02,write_timeout=2,rtscts=False,dsrdtr=False,xonxoff=False)
p.dtr=False; p.rts=False
try:
    p.reset_input_buffer(); p.write(raw); p.flush()
    end=time.monotonic()+a.timeout; rx=bytearray()
    while time.monotonic()<end and len(rx)<a.bytes:
        rx += p.read(max(1,p.in_waiting))
    print('TX:', raw.hex(' ').upper())
    print('RX:', bytes(rx).hex(' ').upper())
finally:
    p.close()
