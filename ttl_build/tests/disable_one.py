import serial
p=serial.Serial('COM3',115200,timeout=1,write_timeout=2); p.dtr=False; p.rts=False
p.write(bytes.fromhex('ff ff 08 01 01 01 00 55')); p.flush(); print(p.read(5).hex()); p.close()
