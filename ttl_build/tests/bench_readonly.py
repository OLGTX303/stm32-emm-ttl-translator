"""Read-only COM3 qualification using the original host wire format.
Does not enable a motor or submit a motion command.
"""
import argparse
import ast
import json
from pathlib import Path
import struct
import time
import serial

root=Path(__file__).resolve().parents[2]
text=(root/'software/cube_motion.py').read_text(encoding='utf-8-sig')
tree=ast.parse(text)
names={'crc8','build_command_frame','parse_stat_response'}
tree.body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names]
api=dict(struct=struct)
exec(compile(tree,str(root/'software/cube_motion.py'),'exec'),api)
parser=argparse.ArgumentParser()
parser.add_argument('--port',default='COM3')
args=parser.parse_args()
results=[]
port=serial.Serial()
port.port=args.port; port.baudrate=115200; port.timeout=0.02; port.write_timeout=1
port.dtr=False; port.rts=False
port.open()
try:
    port.reset_input_buffer()
    for ident in (1,2,3,4):
        packet=api['build_command_frame'](1,[ident],[0],[b''])
        start=time.monotonic()
        port.write(packet); port.flush()
        rx=bytearray()
        while time.monotonic()-start<1.1:
            rx+=port.read(max(1,port.in_waiting))
            header=rx.find(b'\xff\xff')
            if header>=0:
                del rx[:header]
                if len(rx)>=15: break
        item=dict(id=ident,tx=packet.hex(' '),rx=rx.hex(' '),
                  latency_ms=round((time.monotonic()-start)*1000,3))
        try:
            item['status']=api['parse_stat_response'](bytes(rx[:15]))
        except Exception as e:
            item['error']=str(e)
        results.append(item)
finally:
    port.close()
result=dict(port=args.port,baud=115200,kind='physical read-only serial test',results=results)
(root/'ttl_build/build/bench_readonly.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2,ensure_ascii=False))
