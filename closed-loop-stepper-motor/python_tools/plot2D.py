#!/usr/bin/python3
import sys
import csv
import matplotlib.pyplot as plt

def plot(filename):
    # 数据读取与验证
    data = []
    with open(filename) as f:
        reader = csv.reader(f)          # 1. 创建reader对象
        headers = next(reader)          # 2. 正确读取表头
        
        for row in reader:              # 3. 继续读取数据行
            if len(row) == 5:           # 4. 改为检查5列
                try:
                    data.append(list(map(float, row)))
                except ValueError:
                    continue

    # 创建3个垂直排列的子图
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))
    fig.suptitle('5 Channel Data Plot')
    
    # 绘制第一个子图：前两列数据
    ax1.plot([row[0] for row in data], label=headers[0])
    ax1.plot([row[1] for row in data], label=headers[1])
    ax1.grid(True)
    ax1.legend()
    
    # 绘制第二个子图：中间两列数据
    ax2.plot([row[2] for row in data], label=headers[2])
    ax2.plot([row[3] for row in data], label=headers[3])
    ax2.grid(True)
    ax2.legend()
    
    # 绘制第三个子图：最后一列数据
    ax3.plot([row[4] for row in data], label=headers[4], color='purple')
    ax3.grid(True)
    ax3.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) == 2:
        plot(sys.argv[1])
    else:
        print("Usage: plot2D.py <datafile.csv>")