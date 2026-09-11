"""Exercise production C using frames emitted by the UNMODIFIED host software.
The simulator models serial wire time and finite-speed ideal motor motion.
It cannot establish physical X42S tracking accuracy, force or homing safety.
"""
import ast
import ctypes as C
import heapq
import logging
from pathlib import Path
import struct
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'software/cube_motion.py').read_text(encoding='utf-8-sig')
tree = ast.parse(source)
# Execute original declarations/functions, omitting serial imports and __main__.
nodes = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom, ast.If))]
sw = dict(struct=struct, time=time, logging=logging, sys=sys, __name__='original_cube_motion')
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(ROOT/'software/cube_motion.py'), 'exec'), sw)
dll = C.CDLL(str(ROOT / 'ttl_build/build/translator_test.dll'))
for name in ('test_poll',):
    getattr(dll, name).argtypes = [C.c_uint32]
for name in ('test_host', 'test_motor'):
    getattr(dll, name).argtypes = [C.c_uint8, C.c_uint32]
for name in ('test_bus', 'test_host_out'):
    getattr(dll, name).argtypes = [C.POINTER(C.c_uint8)]
    getattr(dll, name).restype = C.c_uint16
dll.test_carry.restype = C.c_float
dll.test_start.restype = C.c_uint32
dll.test_position.restype = C.c_int32
dll.test_target.restype = C.c_int32
dll.test_finish.restype = C.c_uint32
dll.tr_counts_to_pulses.argtypes = [C.c_uint8, C.c_int32]
dll.tr_counts_to_pulses.restype = C.c_int64
buf = (C.c_uint8 * 4096)()

def take(name):
    n = getattr(dll, name)(buf)
    return bytes(buf[:n])

def frame(ids, typ, data=None):
    return sw['build_command_frame'](len(ids), ids, [typ]*len(ids), data or [b'']*len(ids))

class Simulator:
    def __init__(self):
        dll.test_reset()
        self.t = 0
        self.events = []
        self.history = []
        self.positions = [0.0]*4
        self.targets = [0.0]*4
        self.rpm = [0]*4
        self.enabled = [False]*4
        self.current = [2500]*4
        self.flags = [0]*4
        self.missing = set()
        self.reject = None
        self.limit = {}
        self.linked_limits = False
        self.freeze = set()
        self.output = bytearray()
        self.config_bad = False
        self.no_temperature = False
        self.stale_notifications = False

    def schedule(self, at, payload):
        # Deliver each byte at real 115200 8N1 spacing.
        for j, b in enumerate(payload):
            heapq.heappush(self.events, (at + (j+1)*87, b))

    def response(self, ident, code, payload):
        if ident in self.missing:
            return
        if self.reject == (ident, code):
            payload = b'\xe2'
        self.schedule(self.t+500, bytes([ident, code])+payload+b'\x6b')

    def command(self, packet, batch=False):
        ident, code = packet[:2]
        if ident == 0:
            ids = range(1,5)
        else:
            ids = [ident]
        if ident in self.missing:
            return
        if code == 0xFD:
            j = ident-1
            sign = -1 if packet[2] else 1
            self.targets[j] = sign*int.from_bytes(packet[6:10], 'big')*16384/3200
            self.rpm[j] = int.from_bytes(packet[3:5], 'big')
            if self.stale_notifications:
                self.schedule(self.t+50, bytes([1, 0xFD, 0x9F, 0x6B]))
        elif code == 0x45:
            self.current[ident-1] = int.from_bytes(packet[4:6], 'big')
        elif code == 0xF3:
            for ident2 in ids:
                self.enabled[ident2-1] = bool(packet[3])
                if not packet[3]:
                    self.targets[ident2-1] = self.positions[ident2-1]
        elif code == 0xFE:
            for ident2 in ids:
                self.targets[ident2-1] = self.positions[ident2-1]
        elif code == 0x42:
            r = bytearray.fromhex('21 15 19 02 02 02 00 10 01 00 04 B0 0B 80 0F A0 05 07 01 00 01 01 00 08 08 98 07 D0 00 08')
            r[18] = ident
            if self.config_bad:
                r[0] = 37
            self.response(ident, code, r)
            return
        elif code == 0x1F:
            self.response(ident, code, bytes.fromhex('00 C8 10 02'))
            return
        elif code == 0x36:
            v = round(self.positions[ident-1]*4)
            self.response(ident, code, bytes([v<0])+abs(v).to_bytes(4,'big'))
            return
        elif code == 0x3A:
            j = ident-1
            reached = abs(self.positions[j]-self.targets[j]) < 1
            self.response(ident, code, bytes([int(self.enabled[j]) | (2 if reached else 0) | self.flags[j]]))
            return
        elif code == 0x39:
            if self.no_temperature:
                self.response(ident, 0, b'\xee'); return
            self.response(ident, code, bytes([1, 35]))
            return
        elif code == 0x24:
            self.response(ident, code, (24000).to_bytes(2,'big'))
            return
        if not batch and packet[0]:
            self.response(packet[0], code, b'\x02')

    def consume_bus(self, p):
        self.history.append((self.t, p))
        # Motor receives complete packet at tx_end.
        if p[:2] == b'\x00\xaa':
            assert int.from_bytes(p[2:4],'big') == len(p)
            for i in range(4, len(p)-1, 13):
                assert p[i+1] == 0xFD and p[i+12] == 0x6B
                self.command(p[i:i+13], batch=True)
            self.response(1, 0xFD, b'\x02')
        else:
            self.command(p)

    def step(self, dt=100):
        self.t += dt
        for j in range(4):
            if self.enabled[j] and j+1 not in self.freeze:
                distance = self.targets[j]-self.positions[j]
                travel = self.rpm[j]*16384/60*dt/1e6
                self.positions[j] += max(-travel, min(travel, distance))
                if j+1 in self.limit:
                    limit = self.limit[j+1]
                    if self.linked_limits and j in (0,2):
                        limit += self.positions[j+1]/2
                    self.positions[j] = max(limit, self.positions[j])
        while self.events and self.events[0][0] <= self.t:
            at, item = heapq.heappop(self.events)
            if isinstance(item, bytes):
                self.consume_bus(item)
            else:
                dll.test_motor(item, at)
        dll.test_poll(self.t)
        p = take('test_bus')
        if p:
            tx = dll.test_tx_end() & 0xffffffff
            tx = self.t + ((tx-(self.t & 0xffffffff)) & 0xffffffff)
            heapq.heappush(self.events, (tx, p))
        self.output += take('test_host_out')

    def run(self, us):
        end = self.t+us
        while self.t < end:
            self.step()

    def send(self, data):
        for b in data:
            dll.test_host(b, self.t)
            self.step(10)  # host wire: 1 Mbps 8N1
        self.output += take('test_host_out')

    def receive(self, n, timeout=900000):
        end = self.t+timeout
        while len(self.output) < n and self.t < end:
            self.step()
        result = bytes(self.output[:n])
        del self.output[:n]
        return result

    def enable(self, ids=(1,2,3,4), on=True):
        self.send(frame(ids,1,[bytes([on])]*len(ids)))
        return self.receive(5)

    def trap(self, ids, segments, home=False):
        data = [struct.pack('<ihhhB', *s) for s in segments]
        self.send(frame(ids,3 if home else 2,data))
        return self.receive(5)

    def stat(self, ident):
        self.send(frame([ident],0))
        return self.receive(15)

def ok(response):
    return len(response) in (5,15) and response[3] == 0 and sw['crc8'](response[:-1]) == response[-1]

class Tests(unittest.TestCase):
    def setUp(self):
        self.s = Simulator()

    def test_01_enable_status_disable(self):
        self.assertTrue(ok(self.s.enable()))
        r = self.s.stat(1)
        self.assertTrue(ok(r), r.hex())
        self.assertEqual(sw['parse_stat_response'](r)[2:], [35,0,24000])
        self.assertTrue(ok(self.s.enable(on=False)))
        self.assertEqual(list(self.s.enabled), [False]*4)

    def test_02_crc_and_resynchronization(self):
        bad = bytearray(frame([1],1,[b'\x01']))
        bad[-1] ^= 1
        self.s.send(b'junk'+bad)
        self.s.run(25000)
        self.assertEqual(self.s.history, [])
        self.assertTrue(ok(self.s.enable([1])))

    def test_03_invalid_atomic_request(self):
        self.assertTrue(ok(self.s.enable([1,2])))
        before = len(self.s.history)
        r = self.s.trap([1,2], [(1000,0,100,20,100),(2000,0,100,20,101)])
        self.assertFalse(ok(r))
        self.assertEqual(dll.test_count(0), 0)
        self.assertEqual(dll.test_count(1), 0)
        self.assertFalse(any(p[1]==0xAA for _,p in self.s.history[before:]))

    def test_04_signed_rounding(self):
        for x in [-2147483648,-16384,-8192,-1,0,1,8192,16384,2147483647]:
            expected = (abs(x)*3200+8192)//16384 * (-1 if x<0 else 1)
            self.assertEqual(dll.tr_counts_to_pulses(0,x), expected)
        self.assertTrue(ok(self.s.enable([1])))
        self.assertTrue(ok(self.s.trap([1],[(-1000,0,100,20,100)])))
        self.s.run(400000)
        self.assertEqual(dll.test_fault(),0)
        self.assertLess(abs(self.s.positions[0]+1000),6)

    def test_05_group_start(self):
        self.assertTrue(ok(self.s.enable([1,2])))
        self.assertTrue(ok(self.s.trap([1,2],[(4096,0,300,80,100),(8192,0,600,160,100)])))
        self.s.run(30000)
        self.assertEqual(dll.test_start(0),dll.test_start(1))
        self.s.run(450000)
        self.assertEqual(dll.test_fault(),0)
        self.assertEqual(dll.test_count(0),0)
        self.assertEqual(dll.test_count(1),0)

    def test_06_queue_capacity(self):
        self.assertTrue(ok(self.s.enable([1])))
        for _ in range(16):
            self.assertTrue(ok(self.s.trap([1],[(100000,0,10,20,100)])))
        r = self.s.trap([1],[(100000,0,10,20,100)])
        self.assertEqual(r[3],3)
        self.assertEqual(dll.test_count(0),16)

    def test_07_nonzero_velocity_chain(self):
        self.assertTrue(ok(self.s.enable([1])))
        for s in [(2000,200,400,300,100),(4000,200,400,300,100),(6000,0,400,300,100)]:
            self.assertTrue(ok(self.s.trap([1],[s])))
        self.s.run(400000)
        self.assertEqual(dll.test_fault(),0)
        self.assertEqual(dll.test_count(0),0)
        self.assertLess(abs(self.s.positions[0]-6000),6)

    def test_08_homing_stall(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.s.limit[1] = -1000
        self.assertTrue(ok(self.s.trap([1],[(-10000,0,30,20,15)],True)))
        self.s.run(500000)
        self.assertEqual(dll.test_fault(),0)
        self.assertEqual(dll.test_count(0),0)
        self.assertEqual(self.s.current[0],375)
        self.assertLess(abs(dll.test_position(0)+1000),8)
        self.assertTrue(any(p[1]==0xFE for _,p in self.s.history))
        self.assertFalse(any(p[1]==0x0A for _,p in self.s.history))

    def test_09_clamp_hold(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.s.limit[1] = -500
        self.assertTrue(ok(self.s.trap([1],[(-1000,0,1500,1600,40)])))
        self.s.run(400000)
        self.assertEqual(dll.test_fault(),0)
        self.assertEqual(dll.test_count(0),0)
        self.assertTrue(self.s.enabled[0])
        self.assertEqual(self.s.current[0],1000)

    def test_10_missing_motor(self):
        self.s.missing.add(2)
        r = self.s.enable([2])
        self.assertFalse(ok(r))
        self.assertEqual(dll.test_fault(),6)

    def test_11_motor_rejection(self):
        self.s.reject=(1,0xF3)
        self.assertFalse(ok(self.s.enable([1])))
        self.assertEqual(dll.test_fault(),7)

    def test_12_stale_completion_does_not_finish(self):
        self.s.stale_notifications=True
        self.assertTrue(ok(self.s.enable([1])))
        self.assertTrue(ok(self.s.trap([1],[(10000,0,100,20,100)])))
        self.s.run(30000)
        self.assertNotEqual(dll.test_count(0),0)

    def test_13_disable_cancels_motion(self):
        self.assertTrue(ok(self.s.enable([1,2])))
        self.assertTrue(ok(self.s.trap([1,2],[(10000,0,100,20,100),(20000,0,200,40,100)])))
        self.s.run(30000)
        self.assertTrue(ok(self.s.enable([1,2],False)))
        self.assertEqual(dll.test_count(0)+dll.test_count(1),0)

    def test_14_four_axis_sustained_ramps_expose_bandwidth(self):
        self.assertTrue(ok(self.s.enable()))
        self.assertTrue(ok(self.s.trap([1,2,3,4],[(100000,0,300,1,100)]*4)))
        self.s.run(200000)
        # Four sustained acceleration ramps still require 57-byte streams at
        # 100 Hz. Unlike cruise, they cannot be coalesced without changing motion.
        self.assertIn(dll.test_fault(), [8,9])

    def test_15_fault_on_missing_feedback(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.assertTrue(ok(self.s.trap([1],[(10000,0,100,20,100)])))
        self.s.run(20000)
        self.s.missing.add(1)
        self.s.run(200000)
        self.assertNotEqual(dll.test_fault(),0)

    def test_16_counter_and_software_packets(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.assertEqual(dll.test_commands(0),1)
        r = self.s.stat(1)
        self.assertEqual(sw['parse_stat_response'](r)[0],2)


    def test_17_original_software_homing_and_actions(self):
        sim = self.s
        class Clock:
            @staticmethod
            def time(): return sim.t/1e6
            @staticmethod
            def sleep(seconds): sim.run(round(seconds*1e6))
        class Serial:
            @property
            def in_waiting(self):
                sim.step()
                return len(sim.output)
            def write(self, data): sim.send(data)
            def read(self, n):
                result=bytes(sim.output[:n]); del sim.output[:n]
                return result
        old_time=sw['time']
        sw['time']=Clock
        sim.limit={1:-1000,3:-1000}
        sim.linked_limits=True
        try:
            zeros=sw['cmd_zero'](Serial())
            self.assertIsNotNone(zeros)
            controller=sw['MotionCtrl'](Serial(),*zeros)
            controller.two_finger_init()
            controller.two_finger_clamp()
            controller.move_arm(90,False,100,100,80)
            controller.arm_90_no_load(False)
            controller.arm_90_no_load(True,True)
            for side in ('L','R'):
                for action in ('0','1','+T','-T','*T','+F','-F','*F'):
                    controller.motions([side+action])
                controller.motions([side+'2',side+'+N',side+'0'])
                controller.motions([side+'2',side+'+N'])
            self.assertEqual(dll.test_fault(),0)
        except Exception:
            print('FAULT DETAIL',sim.t,sim.positions,sim.targets,
                  [(dll.test_position(i),dll.test_target(i),dll.test_finish(i)) for i in range(4)],
                  [(t,p.hex()) for t,p in sim.history[-12:]])
            raise
        finally:
            sw['time']=old_time

    def test_18_fragmented_packet_and_embedded_6b(self):
        data=frame([1],1,[b'\x01'])
        self.s.send(data[:3]); self.s.run(5000)
        self.assertEqual(self.s.history,[])
        self.s.send(data[3:])
        self.assertTrue(ok(self.s.receive(5)))
        self.assertTrue(ok(self.s.trap([1],[(0x6b,0,100,20,100)])))
        self.s.run(300000)
        self.assertEqual(dll.test_fault(),0)

    def test_19_deadline_is_a_fault(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.assertTrue(ok(self.s.trap([1],[(100000,0,100,20,100)])))
        self.s.run(30000)
        self.s.step(30000)
        self.assertNotEqual(dll.test_fault(),0)

    def test_20_reject_incompatible_config(self):
        self.s.config_bad=True
        self.assertFalse(ok(self.s.enable([1])))
        self.assertFalse(self.s.enabled[0])

    def test_21_recover_only_after_explicit_disable_all(self):
        self.s.missing.add(2)
        self.assertFalse(ok(self.s.enable([2])))
        self.s.run(30000)
        self.s.missing.clear()
        self.assertTrue(ok(self.s.enable(on=False)))
        self.assertTrue(ok(self.s.enable([1])))

    def test_22_idle_wait_has_fresh_measurement(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.s.run(100000)
        self.s.positions[0]=1234
        response=self.s.stat(1)
        self.assertTrue(ok(response))
        self.assertEqual(sw['parse_stat_response'](response)[3],1234)


    def test_23_single_disable_stops_its_linked_group(self):
        self.assertTrue(ok(self.s.enable([1,2,3])))
        self.assertTrue(ok(self.s.trap([1,2],[(10000,0,100,20,100),(20000,0,200,40,100)])))
        self.s.run(20000)
        self.assertTrue(ok(self.s.enable([1],False)))
        self.assertFalse(self.s.enabled[0])
        self.assertTrue(self.s.enabled[1])
        self.assertTrue(self.s.enabled[2])
        self.assertEqual(dll.test_count(0)+dll.test_count(1),0)
        self.assertTrue(any(p[:2]==b'\x02\xfe' for _,p in self.s.history))

    def test_24_reenable_does_not_change_clamp_current(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.assertTrue(ok(self.s.trap([1],[(10000,0,100,20,40)])))
        self.s.run(30000)
        self.assertEqual(self.s.current[0],1000)
        self.assertTrue(ok(self.s.enable([1])))
        self.assertEqual(self.s.current[0],1000)

    def test_25_profile_matches_original_model(self):
        import math
        text=(ROOT/'software/verify_arm_finger_linkage.py').read_text(encoding='utf-8-sig')
        original=ast.parse(text)
        original.body=[n for n in original.body if isinstance(n,ast.ClassDef) and n.name=='Trapezoid']
        ns=dict(math=math,rpm_to_speed=16384/5000/60)
        exec(compile(original,'verify_arm_finger_linkage.py','exec'),ns)
        class Profile(C.Structure):
            _fields_=[(n,C.c_float) for n in ('v0','v1','vc','a','ta','tv','td','duration','distance')]
        dll.tr_profile.argtypes=[C.POINTER(Profile),C.c_int64,C.c_float,C.c_int16,C.c_int16,C.c_int16]
        dll.tr_profile.restype=C.c_bool
        dll.tr_profile_position.argtypes=[C.POINTER(Profile),C.c_float]
        dll.tr_profile_position.restype=C.c_float
        for distance,v0,v1,vmax,a in [(8192,0,0,700,600),(-8192,0,0,400,160),(2000,200,200,400,300),(500,0,0,1500,1600)]:
            ref=ns['Trapezoid']()
            ref.calculate_motion(0,distance,v0,v1,vmax,a)
            p=Profile()
            self.assertTrue(dll.tr_profile(C.byref(p),distance,v0,v1,vmax,a))
            self.assertAlmostEqual(p.duration,ref.T/5000,places=5)
            for fraction in (0,0.1,0.25,0.5,0.75,0.9,1):
                # Independent original model samples in 5 kHz tick units.
                expected=ref.get_pos(ref.T*fraction)[0]
                actual=dll.tr_profile_position(C.byref(p),p.duration*fraction)
                self.assertLess(abs(actual-expected),0.02)


    def test_26_counter_wrap(self):
        self.assertTrue(ok(self.s.enable([1])))
        dll.test_set_counter(0,65535)
        r=self.s.stat(1)
        self.assertEqual(sw['parse_stat_response'](r)[0],0)

    def test_27_clock_wrap(self):
        dll.test_advance_clock.argtypes=[C.c_uint32]
        self.s.t=0xffff0000
        dll.test_advance_clock(self.s.t)
        # The simulator keeps absolute timestamps; C receives wrapping uint32.
        self.assertTrue(ok(self.s.enable([1])))
        self.assertTrue(ok(self.s.trap([1],[(1000,0,100,20,100)])))
        # Translate outgoing completion time to the simulator's extended timeline.
        self.s.run(200000)
        self.assertEqual(dll.test_fault(),0)

    def test_28_bad_length_unknown_and_duplicate_ids(self):
        self.s.send(b'\xff\xff\x03')
        self.assertTrue(ok(self.s.enable([1])))
        for p in (frame([1],9),frame([1,1],1,[b'\x00',b'\x00'])):
            before=list(self.s.enabled)
            self.s.send(p)
            self.assertFalse(ok(self.s.receive(5)))
            self.assertEqual(self.s.enabled,before)

    def test_29_underrun_is_not_a_stop_at_waypoint(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.assertTrue(ok(self.s.trap([1],[(1000,100,200,100,100)])))
        self.s.run(150000)
        self.assertEqual(dll.test_fault(),13)

    def test_30_transport_error_aborts_and_latches(self):
        self.assertTrue(ok(self.s.enable([1])))
        self.assertTrue(ok(self.s.trap([1],[(10000,0,100,20,100)])))
        self.s.run(20000)
        dll.tr_uart_fault()
        self.s.run(30000)
        self.assertEqual(dll.test_count(0),0)
        self.assertEqual(dll.test_fault(),12)
        self.assertFalse(self.s.enabled[0])


    def test_31_four_motors_complete_with_cruise_coalescing(self):
        self.assertTrue(ok(self.s.enable()))
        self.assertTrue(ok(self.s.trap([1,2,3,4],[(100000,0,300,80,100)]*4)))
        self.s.run(2000000)
        self.assertEqual(dll.test_fault(),0)
        self.assertEqual(sum(dll.test_count(i) for i in range(4)),0)
        self.assertLess(dll.test_streams(),20)
        for p in self.s.positions:
            self.assertLess(abs(p-100000),6)


    def test_32_explicitly_unsupported_temperature_is_unknown(self):
        self.s.no_temperature=True
        self.assertTrue(ok(self.s.enable([1])))
        r=self.s.stat(1)
        self.assertTrue(ok(r))
        self.assertEqual(sw['parse_stat_response'](r)[2],-128)
        self.assertEqual(sw['parse_stat_response'](r)[4],24000)
        self.assertEqual(dll.test_fault(),0)

if __name__ == '__main__':
    sw['logger'].setLevel(logging.ERROR)
    unittest.main(verbosity=2)
