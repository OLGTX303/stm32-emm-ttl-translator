#!/usr/bin/python3
# 程序设计要求
# 程序1 serial_debug.py
# 1、打开用户指定的串口文件
# 2、设计一个类似minicom的交互式终端，用户可输入指令，设备会回复数据，回显这些数据，例如：
# 提示用户输入：CMD>
# 用户输入：CMD>hello
# 用户输入回车后发送数据

# 3、当设备回传的数据中出现START: STOP:包围的数据时，记录这些数据，保存在当前目录下，以当前日期时间命名，并启动绘图功能（程序2）
# START:后面的表头也要保存，用于绘图时标注曲线
# stepper# debug
# START:speed_target,speed_read,iq_target,iq,power_voltage
# 0,-1,9,8,2881
# 0,-1,9,4,2881
# 0,-1,9,3,2881
# 0,-1,9,6,2881
# 0,-1,9,6,2882
# 0,-1,10,4,2881
# 中间省略，最多有4096条数据，也可能会少一些
# 0,-49,9,60,2881
# 0,-12,130,4,2881
# STOP:

# 程序2 plot2D.py：
# 绘图功能应当绘制5条曲线，曲线1,2出现在一个图里，曲线3,4出现在另一个图里，曲线5出现在最后一个图里，所有图出现在一个窗口里
# 曲线要标注，例如speed_target,speed_read,iq_target,iq,power_voltage
# 图表里不要出现中文，以免在某些环境下出现兼容性问题。

import sys
import serial
from serial.tools import list_ports
import threading
from queue import Queue
import re
import time
import csv
import subprocess
from datetime import datetime
import queue 

class SerialMonitor:
    def __init__(self, port):
        self.ser = self.connect_serial(port)
        self.data_buffer = []
        self.running = True
        self.plot_queue = Queue()
        self.realtime_buffer = []
        self.in_data_block = False  # 新增数据块状态标志
        self.headers = []  # 新增表头存储
        # 启动数据处理线程
        self.process_thread = threading.Thread(target=self._process_data)
        self.process_thread.start()
    def _process_data(self):
        """线程安全的数据处理方法"""
        while self.running:  # 修改循环判断条件
            try:
                # 接收元组格式数据(headers, data)
                headers, data = self.plot_queue.get(timeout=0.5)
                if data is None or headers is None:
                    break
                    
                # 数据保存与绘图逻辑
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{timestamp}.csv"
                
                # 保存时同时使用表头
                with open(filename, 'w') as f:
                    writer = csv.writer(f)
                    writer.writerow(headers)  # 写入表头
                    writer.writerows(data)
                
                subprocess.Popen(['python3', 'plot2D.py', filename])
                print(f"Saved: {filename}")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Data processing error: {str(e)}")

    def process_line(self, line):
        """增强型数据处理方法"""
        line = line.strip()
        if line.startswith('START:'):
            self.in_data_block = True  # 必须设置数据块状态
            self.headers = line.split(':')[1].strip().split(',')
            self.data_buffer = []
        elif line.startswith('STOP:'):
            self.in_data_block = False
            if self.data_buffer:
                # 需要同时传递表头和数据
                self.plot_queue.put( (self.headers, self.data_buffer.copy()) )
            self.data_buffer = []
        elif self.in_data_block:
            if re.match(r'^([+-]?\d+(\.\d+)?,){4}[+-]?\d+(\.\d+)?$', line):
                self.data_buffer.append(line.split(','))
            else:
                print(f"Ignored invalid data: {line}")

    def connect_serial(self, port):
        """尝试不同波特率连接串口"""
        for baud in [115200, 9600, 57600, 921600]:
            try:
                ser = serial.Serial(
                    port=port,
                    baudrate=baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1
                )
                print(f"成功连接 {port} @ {baud}bps")
                return ser
            except serial.SerialException:
                continue
        raise RuntimeError("无法打开串口")

    def start_monitoring(self):
        """启动数据监听线程"""
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def read_serial(self):
        """持续读取串口数据"""
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"<<< {line}")  # 实时回显
                    self.process_line(line)
            except serial.SerialException:
                break

    def send_command(self, cmd):
        """发送命令到串口"""
        full_cmd = f"{cmd}\r".encode()
        print(f">>> {cmd}")
        self.ser.write(full_cmd)
        self.ser.flush()

    def stop(self):
        """安全的线程终止方法"""
        self.running = False
        # 先标记运行状态为False
        time.sleep(0.1)  # 留出线程响应时间
        
        # 清空队列防止残留数据
        while not self.plot_queue.empty():
            try:
                self.plot_queue.get_nowait()
            except queue.Empty:
                break
                
        # 发送终止信号
        self.plot_queue.put(None)  # 保证线程退出
        self.process_thread.join(timeout=1.0)

def list_serial_ports():
    """列出可用串口"""
    ports = list_ports.comports()
    if not ports:
        print("未检测到可用串口设备")
        return
    print("可用串口设备:")
    for port in ports:
        print(f"  {port.device} - {port.description}")

def interactive_shell(monitor):
    """交互式命令行界面"""
    print("\n交互式串口终端（输入 'exit' 退出）")
    try:
        while True:
            # 处理用户输入
            cmd = input("CMD> ").strip()
            if not cmd:
                continue
            if cmd.lower() in ['exit', 'quit']:
                break
            monitor.send_command(cmd)
            
    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("使用方法: python serial_debug.py <串口设备路径>")
        print("示例:")
        print("  python serial_debug.py /dev/ttyACM0")
        list_serial_ports()
        sys.exit(1)

    try:
        monitor = SerialMonitor(sys.argv[1])
        monitor.start_monitoring()
        interactive_shell(monitor)
    except Exception as e:
        print(f"错误: {str(e)}")
        sys.exit(1)



