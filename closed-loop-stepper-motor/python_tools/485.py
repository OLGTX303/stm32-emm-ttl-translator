#!/usr/bin/python3
import serial
import struct
import time
import sys
from serial.tools import list_ports

# ------------------------------- 以下是485通信代码 -------------------------------
def crc8(datagram):
    """计算CRC8校验值，多项式0x07，初始值0x00"""
    crc = 0
    for byte in datagram:
        current_byte = byte
        for _ in range(8):
            bit = (crc >> 7) ^ (current_byte & 0x01)
            if bit:
                crc = (crc << 1) ^ 0x07
            else:
                crc = (crc << 1)
            crc &= 0xFF  # 保持8位
            current_byte >>= 1
    return crc

def build_command_frame(node_id, command_types, data):
    """
    ===== 主机→电机控制器数据帧格式 =====
    [字头] + [数据长度] + [指令类型 + ID号] + [数据] + [CRC8]

    [01字头]             uint16_t 固定值0xFFFF（2字节）
    [2 数据长度]          uint8_t len（1字节），指数据包的总长度，含0xFFFF和校验和
    [3 指令类型 + ID号]   低4bit：取值范围1-4，0用于广播
                        高4bit：指令类型
    [n-1 数据]           uint8_t data[250] 最大250字节
    [n CRC8]            uint8_t 校验值（含字头）
    注：每个电机仅处理ID匹配的指令块，其他忽略
    """

    # 计算数据长度字段
    data_length = 5 + len(data)
    if data_length > 255:
        raise ValueError("数据长度超过最大限制255字节")
        
    # 构造完整数据帧
    frame = b'\xff\xff'  # 字头
    frame += bytes([data_length])
    frame += bytes([(command_types<<4) | node_id])
    frame += bytes(data)
    crc_value = crc8(frame)
    frame += bytes([crc_value])
    
    return frame

def receive_response(ser, expected_length=15, timeout=1):
    """接收响应数据，并校验结构"""
    start_time = time.time()
    buffer = bytearray()
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            buffer += ser.read(ser.in_waiting)
            # 查找字头0xFF 0xFF
            while len(buffer) >= 2:
                pos = buffer.find(b'\xff\xff')
                if pos == -1:
                    # 没有找到字头，保留最后一个字节继续查找
                    buffer = buffer[-1:] if buffer else bytearray()
                    break
                else:
                    # 找到字头，截取后续数据
                    buffer = buffer[pos:]
                    if len(buffer) < expected_length:
                        # 数据不足，继续等待
                        break
                    else:
                        # 提取完整帧
                        frame = buffer[:expected_length]
                        buffer = buffer[expected_length:]
                        return frame
        time.sleep(0.001)
    return None

def parse_stat_response(frame):
    """解析查询指令的响应数据"""
    # 修正1: 响应帧长度应为17字节
    if len(frame) != 17:
        raise ValueError(f"响应帧长度必须为17字节，实际收到{len(frame)}字节")
    
    # 提取数据部分和CRC（前16字节为数据，最后1字节为CRC）
    data_part = frame[0:16]
    received_crc = frame[16]
    
    # 计算CRC
    calculated_crc = crc8(data_part)
    if calculated_crc != received_crc:
        raise ValueError(f"CRC校验失败: 计算值{calculated_crc:02X}, 接收值{received_crc:02X}")
    
    # 检查帧头标志
    if frame[0] != 0xFF or frame[1] != 0xFF:
        raise ValueError("帧头必须为0xFF 0xFF")
    
    # 检查长度标识（包含自身）
    if frame[2] != 17:
        raise ValueError(f"长度标识应为17，实际为{frame[2]}")
    
    # 修正2: 标志位检查位置在frame[3]
    if frame[3] != 0xFF:
        raise ValueError(f"标志位不为0xFF, 实际为{frame[3]:02X}")
    
    # 解析数据字段（小端序）
    cmd_count = frame[4]
    pos_error = struct.unpack('<H', frame[5:7])[0]  # uint16_t
    fifo_status = struct.unpack('<H', frame[7:9])[0]  # uint16_t
    temperature = struct.unpack('b', bytes([frame[9]]))[0]  # int8_t
    pos = struct.unpack('<i', frame[10:14])[0]  # int32_t
    voltage = struct.unpack('<h', frame[14:16])[0]  # int16_t
    
    # 修正3: 返回字段与C代码结构一致
    return {
        'cmd_count': cmd_count,
        'pos_error': pos_error,
        'status': (fifo_status & 0x8000) != 0,
        'fifo_count': fifo_status & 0x7FFF,
        'temperature': temperature,
        'position': pos,
        'voltage': voltage
    }

def parse_other_response(response_frame):
    if len(response_frame) != 5:
        raise ValueError("响应帧长度必须为5字节")
    print(f"接收到响应帧: {response_frame.hex()}")
    # 计算CRC
    calculated_crc = crc8(response_frame[0:4])
    received_crc = response_frame[4]
    if calculated_crc != received_crc:
        raise ValueError(f"CRC校验失败: 计算值{calculated_crc:02X}, 接收值{received_crc:02X}")
    # 解析数据字段
    flag = response_frame[3]
    if flag != 0xff:
        raise ValueError(f"标志位不为0xff, flag={flag}")
    return True


def simple_cmd(ser, node_id, cmd, data_list=[]):
    # 构造指令
    enable_frame = build_command_frame(node_id, cmd, data_list)
    print(f"构建数据帧: {enable_frame.hex()}")
    ser.write(enable_frame)

    # 接收响应
    response_frame = receive_response(ser, 5)
    return parse_other_response(response_frame)

# 使能、禁用编号为id的电机
def cmd_enable(ser, node_id):
    return simple_cmd(ser, node_id, 1)

def cmd_disable(ser, node_id):
    return simple_cmd(ser, node_id, 2)

def cmd_reset(ser, node_id):
    return simple_cmd(ser, node_id, 3)

def cmd_clear_fifo(ser, node_id):
    return simple_cmd(ser, node_id, 4)

def cmd_sync(ser, motor_ids, stop_when_block, half_current_when_block, max_current_percent):
    """
    发送同步控制命令到下位机
    
    参数:
        ser: 串口对象
        motor_ids: 要同步控制的电机ID列表 (如 [1,3] 表示控制电机1和3)
        stop_when_block: 堵转时是否暂停 (True/False)
        half_current_when_block: 堵转时电流是否减半 (True/False)
        max_current_percent: 最大电流百分比 (0-100)
    """
    byte4 = 0
    
    # 设置电机控制位 (bit1-bit4对应电机1-4)
    for motor_id in motor_ids:
        if 1 <= motor_id <= 4:
            byte4 |= (1 << motor_id)  # 对应位设置为1
    
    # 设置堵转暂停标志 (bit7)
    if stop_when_block:
        byte4 |= (1 << 7)
    
    
    # 构造指令
    enable_frame = build_command_frame(0, 5, [byte4])
    print(f"构建数据帧: {enable_frame.hex()}")
    ser.write(enable_frame)

    # print("该指令没有回复")
    

def cmd_fifo(ser, node_id, data_list):
    return simple_cmd(ser, node_id, 6, data_list)

def cmd_stat(ser, node_id):
    # 构造查询指令（0x00）
    stat_frame = build_command_frame(node_id, 0, [])
    print(f"构建数据帧: {stat_frame.hex()}")
    ser.write(stat_frame)

    # 接收并解析响应
    response_frame = receive_response(ser, 17)
    if response_frame:
        print(f"接收到响应帧: {response_frame.hex()}")
        parsed_data = parse_stat_response(response_frame)
        print(f"parsed_data={parsed_data}")
        return parsed_data
    else:
        print("未接收到响应")
        return None

# ------------------------------- 以下是测试程序 -------------------------------
def test_motion(ser):
    #  0xF9 111 110 01 current = 6 
    #  0xE1 111 000 01 current = 0
    block_0 = [
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF9, 0xB6, 0xE2, 0x8A, 0x38, 0xA3, 0x8A, 0x46, 
        0xDD, 0x52, 0x37, 0x23, 0x91, 0xC7, 0x1C, 0x8E, 0x47, 0x1C, 0x72, 0x39, 0x1C, 0x71, 0xC8, 0xE4, 
        0x6E, 0xB7, 0x1C, 0x71, 0xC6, 0xDC, 0x8A, 0x36, 0xE3, 0x51, 0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 
        0xB7, 0x13, 0x71, 0x36, 0xE2, 0x6E, 0x26, 0xE2, 0x6D, 0xC4, 0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB8, 
        0x9B, 0x89, 0xB7, 0x13, 0x71, 0x36, 0xE2, 0x6E, 0x26, 0xE2, 0x6D, 0xC4, 0xDC, 0x4D, 0xC4, 0xDB, 
        0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13, 0x71, 0x37, 0x13, 0x6E, 0x26, 0xE2, 0x6D, 0xC4, 0xDC, 0x4D, 
        0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13,
    ]
    block_1 = [
        0x00, 0x01, 0x3B, 0x1F, 0x00, 0x00, 0x27, 0x00, 0xF9, 0x37, 0x13, 0x6E, 0x26, 0xE2, 0x6D, 0xC4, 
        0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13, 0x71, 0x37, 0x13, 0x6E, 0x26, 0xE2, 
        0x6E, 0x26, 0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13, 0x71, 0x37, 0x13, 0x6E, 
        0x26, 0xE2, 0x6E, 0x26, 0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13, 0x71, 0x37, 
        0x13, 0x6E, 0x26, 0xE2, 0x6E, 0x26, 0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB5, 0x14, 0x4A, 0x26, 0xD3, 
        0x69, 0xA6, 0xD2, 0x6D, 0x26, 0x93, 0x4D, 0x33, 0x0B, 0x69, 0x34, 0x9A, 0x69, 0xA4, 0xA1, 0x69, 
        0xA4, 0xD9, 0x6D, 0x34, 0xD3, 0x69, 0xA6, 0xDA, 
    ]
    block_2 = [
        0x0B, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0x01, 0x00, 0xF9, 0xB5, 0x13, 0x6D, 0x80, 
    ]
    blocks = [block_0, block_1, block_2]
    success = cmd_clear_fifo(ser, 1)
    print("cmd_clear_fifo 执行结果:", "成功" if success else "失败")
    success = cmd_fifo(ser, 1, blocks[0])
    print("cmd_fifo 执行结果:", "成功" if success else "失败")
    cmd_sync(ser, [1], False, True, 100)
    i = 1
    while True:
        stat = cmd_stat(ser, 1)
        print(stat)
        if not stat['status']:
            break
        elif stat['fifo_count'] < 250:
            if i < len(blocks):
                success = cmd_fifo(ser, 1, blocks[i])
                print("cmd_fifo 执行结果:", "成功" if success else "失败")
                i += 1
        time.sleep(0.01)



def list_serial_ports():
    """列出可用串口"""
    ports = list_ports.comports()
    if not ports:
        print("未检测到可用串口设备")
        return
    print("可用串口设备:")
    for port in ports:
        print(f"  {port.device} - {port.description}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    else:
        serial_port = '/dev/cu.usbserial-130'  # 默认值

    baud_rate = 1000000  # 1Mbps
    mc = None
    try:
        with serial.Serial(serial_port, baudrate=baud_rate, bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) as ser:
            print("\n串口连接成功，进入交互模式")
            while True:
                print("\n请选择测试项目（针对编号为1的电机）:")
                print("[1]: 使能电机")
                print("[2]: 禁用电机")
                print("[3]: 查询电机角度")
                print("[4]: 复位控制器")
                print("[5]: 测试S型运动控制")
                print("[q]: 退出程序")
                
                choice = input("请输入选项(1/2/q/...) >> ").strip().lower()
                
                if choice in ['exit', 'quit', 'q']:
                    print("退出程序...")
                    break
                elif choice == '1':
                    success = cmd_enable(ser, 1)
                    print("执行结果:", "成功" if success else "失败")
                elif choice == '2':
                    success = cmd_disable(ser, 1)
                    print("执行结果:", "成功" if success else "失败")
                elif choice == '3':
                    resp = cmd_stat(ser, 1)
                    print("执行结果:", "成功" if resp != None else "失败")
                elif choice == '4':
                    success = cmd_reset(ser, 1)
                    print("执行结果:", "成功" if success else "失败")
                elif choice == '5':
                    test_motion(ser)
                else:
                    print("无效选项，请重新输入")


    except serial.SerialException as e:
        print(f"打开串口 {serial_port} 失败: {str(e)}")
        list_serial_ports()
        sys.exit(1)
    except KeyboardInterrupt:
        print("程序被用户中断")

