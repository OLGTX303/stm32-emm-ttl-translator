"""Read one EMM firmware version through the STM32 TTL translator.

Usage: python ttl_build/tests/read_fw.py COM3 4
"""
import argparse
import time
import serial


def crc8(data):
    crc = 0
    for byte in data:
        for _ in range(8):
            bit = (crc >> 7) ^ (byte & 1)
            crc = ((crc << 1) ^ (0x07 if bit else 0)) & 0xFF
            byte >>= 1
    return crc


def read_frame(port, timeout):
    end = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < end:
        if port.in_waiting:
            buf.extend(port.read(port.in_waiting))
        while len(buf) >= 3:
            start = buf.find(b"\xff\xff")
            if start < 0:
                del buf[:-1]
                break
            del buf[:start]
            length = buf[2]
            if length < 5 or length > 128:
                del buf[0]
                continue
            if len(buf) < length:
                break
            frame = bytes(buf[:length])
            del buf[:length]
            return frame
        time.sleep(0.001)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default="COM3")
    parser.add_argument("motor", nargs="?", type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.motor <= 4:
        parser.error("motor must be 1..4")
    frame = bytearray((0xFF, 0xFF, 7, 1, args.motor, 0x1F))
    frame.append(crc8(frame))
    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1) as port:
        port.reset_input_buffer()
        port.write(frame)
        response = read_frame(port, 1.5)
    if response is None:
        raise SystemExit("timeout: no translator response")
    if crc8(response[:-1]) != response[-1]:
        raise SystemExit("bad response CRC: " + response.hex())
    if len(response) == 5:
        raise SystemExit("translator error flag=%d (raw %s)" % (response[3], response.hex()))
    if len(response) != 9 or response[3] != args.motor:
        raise SystemExit("unexpected response: " + response.hex())
    print("motor %d: fw=%d (0x%04X), hw_series=%d, hw_type=%d, hw_version=%d" % (
        response[3], (response[4] << 8) | response[5],
        (response[4] << 8) | response[5], response[6] >> 4,
        response[6] & 0x0F, response[7]))


if __name__ == "__main__":
    main()
