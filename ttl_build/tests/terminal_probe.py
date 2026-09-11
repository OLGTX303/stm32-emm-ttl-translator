import serial,time
p=serial.Serial('COM3',115200,timeout=.5,write_timeout=2); p.dtr=False;p.rts=False
for hx in ('ff ff 08 01 01 01 00 55','ff ff 07 01 01 00 4f'):
    try:
        p.reset_input_buffer(); p.write(bytes.fromhex(hx)); p.flush(); print(hx,'=>',p.read(32).hex(' '))
    except Exception as e: print(hx,'=> ERROR',e)
p.close()
