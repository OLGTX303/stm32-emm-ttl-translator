"""Small physical smoke test: enable four axes, move motor 1 by 100 host counts, disable."""
import ast, json, struct, time
from pathlib import Path
import serial

root=Path(__file__).resolve().parents[2]
tree=ast.parse((root/'software/cube_motion.py').read_text(encoding='utf-8-sig'))
tree.body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in {'crc8','build_command_frame','parse_stat_response'}]
api={'struct':struct}; exec(compile(tree,'cube_motion.py','exec'),api)
port=serial.Serial('COM3',115200,timeout=0.03,write_timeout=1)
port.dtr=False; port.rts=False
def transact(packet, size, timeout=1.2):
    port.reset_input_buffer(); t=time.monotonic(); port.write(packet); port.flush(); b=bytearray()
    while time.monotonic()-t<timeout:
        b += port.read(max(1,port.in_waiting))
        h=b.find(b'\xff\xff')
        if h>=0:
            del b[:h]
            if len(b)>=size: return bytes(b[:size]), round((time.monotonic()-t)*1000,3)
    return bytes(b), round((time.monotonic()-t)*1000,3)
out=[]
try:
    en=api['build_command_frame'](4,[1,2,3,4],[1,1,1,1],[b'\x01']*4)
    rx,ms=transact(en,5); out.append({'step':'enable','tx':en.hex(' '),'rx':rx.hex(' '),'latency_ms':ms})
    move=api['build_command_frame'](1,[1],[2],[struct.pack('<ihhhB',100,0,30,20,10)])
    rx,ms=transact(move,5); out.append({'step':'move_ack','tx':move.hex(' '),'rx':rx.hex(' '),'latency_ms':ms})
    samples=[]
    for _ in range(18):
        time.sleep(.05)
        q=api['build_command_frame'](1,[1],[0],[b''])
        rx,ms=transact(q,15)
        item={'rx':rx.hex(' '),'latency_ms':ms}
        try: item['status']=api['parse_stat_response'](rx)
        except Exception as e: item['error']=str(e)
        samples.append(item)
    out.append({'step':'samples','items':samples})
finally:
    try:
        dis=api['build_command_frame'](4,[1,2,3,4],[1,1,1,1],[b'\x00']*4)
        rx,ms=transact(dis,5); out.append({'step':'disable','tx':dis.hex(' '),'rx':rx.hex(' '),'latency_ms':ms})
    except Exception as e:
        out.append({'step':'disable','error':str(e)})
    port.close()
result={'port':'COM3','baud':115200,'kind':'physical motion smoke test','steps':out}
(root/'ttl_build/build/bench_motion.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
