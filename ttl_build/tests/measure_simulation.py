"""Report serial-time simulation results; these are NOT physical measurements."""
import json
from pathlib import Path
from test_translator import Simulator, dll, ok

rows=[]
for n in range(1,5):
    s=Simulator()
    assert ok(s.enable())
    assert ok(s.trap(list(range(1,n+1)),[(100000,0,300,80,100)]*n))
    start=s.t
    latency=[]
    while s.t-start<400000:
        before=s.t
        response=s.stat(1)
        latency.append((s.t-before)/1000)
        if not ok(response):
            break
    rows.append(dict(active_axes=n, elapsed_ms=(s.t-start)/1000,
                     max_status_latency_ms=max(latency),
                     stream_packets=dll.test_streams(), fault=dll.test_fault(),
                     max_tick_lateness_us=dll.test_late(),
                     stream_wire_ms=(5+13*n)*10/115200*1000))
result=dict(kind='SIMULATION - not bench measurements',
            assumptions=dict(motor_reply_processing_us=500,loop_poll_us=100,
                             uart_format='8N1',motor_baud=115200,host_baud=1000000,
                             ideal_motor_model=True), cases=rows)
target=Path(__file__).resolve().parents[1]/'build/simulation_metrics.json'
target.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
