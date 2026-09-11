"""Run a user-supplied cube-motion sequence through unchanged software code."""
import sys, time
from pathlib import Path
import serial

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'software'))
import cube_motion as cm

SEQUENCE = """L*T R*T L-T L+T R-T R+T L1 R*F L0 R1 L*F R0 L1 R+F L0 L1 R-F L0 R1 L+F R0 R1 L-F R0 L2 L+N L0 R2 R+N L+F R0 L2 L+N R+F L0 R2 R+N L-F R0 L2 L+N R-F L0 L2 L+N R*F L0 R2 R+N R0 R2 R+N L*F R0""".split()

port_name=sys.argv[1] if len(sys.argv)>1 else 'COM3'
ser=serial.Serial(port_name, baudrate=cm.BAUD_RATE, bytesize=serial.EIGHTBITS,
                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                  timeout=1)
try:
    # Allow the translator's four startup 0x9A origin triggers to be issued.
    time.sleep(0.25)
    # Enable one axis at a time so a bad EMM address/configuration is visible.
    for ident in (1,2,3,4):
        try:
            ok=cm.cmd_enable(ser,[ident],1)
            print('enable %d:' % ident, ok)
        except Exception as exc:
            raise RuntimeError('enable failed for motor %d; zeroing was not started: %s' % (ident, exc))
    zero=cm.cmd_zero(ser)
    print('zero:', zero)
    mc=cm.MotionCtrl(ser,zero[0],zero[1],zero[2],zero[3])
    start=time.monotonic()
    mc.motions(SEQUENCE)
    print('completed %.2fs (%d tokens)' % (time.monotonic()-start,len(SEQUENCE)))
finally:
    try:
        try: print('disable:', cm.cmd_enable(ser,[1,2,3,4],0))
        except Exception as exc: print('disable error:', exc)
    finally: ser.close()
