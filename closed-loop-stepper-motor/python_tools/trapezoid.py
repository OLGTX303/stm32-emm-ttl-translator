
import numpy as np
import matplotlib.pyplot as plt
import math
# 情景1，能加速到最高速度vmax：匀加速(Ta)----匀速(Tv)----匀减速(Td)
# 情景2，不能加速到最高速度：匀加速(Ta)----匀减速(Td)
# x0   初始位置
# x1   目标位置
# v0   初始速度
# v1   目标速度
# vmax 允许的最大速度
# a    加速度的绝对值
class Trapezoid:
    # 改写为C语言实现时，异常处理用打印报错并返回错误代码的方式替代，全部浮点运算使用float
    def calculate_motion(self, x0, x1, v0, v1, vmax, a):
        self.x0 = x0
        self.x1 = x1
        self.v0 = v0
        self.v1 = v1
        vmax = abs(vmax)
        a = abs(a)
        
        # 校验速度是否超过允许范围（考虑方向）
        if vmax < 1e-6:
            raise ValueError("过小的最大速度")
        if a < 1e-6:
            raise ValueError("过小的加速度")
        if abs(v0) > vmax or abs(v1) > vmax:
            raise ValueError("初始速度或目标速度超过最大允许速度")

        # 运动方向判断
        S = x1 - x0  # 总位移
        direction = 1 if S >= 0 else -1  # 1:正方向 -1:负方向
        
        # 运动参数初始化
        self.a_accel = a * direction  # 加速阶段加速度
        self.a_decel = -a * direction # 减速阶段加速度
        signed_vmax = vmax * direction  # 最大速度

        # 计算理论加速/减速时间
        Ta_max = (signed_vmax - v0) / self.a_accel
        Td_max = (v1 - signed_vmax) / self.a_decel
        
        # 计算理论位移
        Sa_max = (signed_vmax**2 - v0**2) / (2 * self.a_accel)
        Sd_max = (v1**2 - signed_vmax**2) / (2 * self.a_decel)
        S_needed = Sa_max + Sd_max

        # 判断实际位移是否足够
        Tv = (S - S_needed) / signed_vmax
        if Tv >= 0:
            # 情景1：能加速到最大速度
            self.Ta = Ta_max
            self.Td = Td_max
            self.Tv = Tv
            self.v_c = signed_vmax
            self.T = self.Ta + self.Tv + self.Td
        else:
            # 情景2：无法达到最大速度 
            # 计算中间速度v_c（考虑方向）
            v_c_sq = (2 * a * abs(S) + v0**2 + v1**2) / 2
            v_c_abs = math.sqrt(v_c_sq)
            v_c = v_c_abs * direction 
            # 重新计算加速/减速时间
            self.Ta = (v_c - v0) / self.a_accel
            self.Td = (v1 - v_c) / self.a_decel
            self.Tv = 0
            self.v_c = v_c
            self.T = self.Ta + self.Td

        if self.Ta < 0 or self.Td < 0:
            raise ValueError("计算得到负时间，参数错误")
        
        self.x_acc_end = self.x0 + self.v0 * self.Ta + 0.5 * self.a_accel * self.Ta**2
        self.x_vel_end = self.x_acc_end + self.v_c * self.Tv

    # 改写为C语言实现时，只需要返回x，不需要a和v，需要仔细优化，缩短运行时间
    def get_pos(self, t):
        if t < 0:
            a = 0
            v = self.v0
            x = self.x0
        elif t > self.T:
            a = 0
            v = self.v1
            x = self.x1
        elif t <= self.Ta:
            dt = t
            a = self.a_accel
            v = self.v0 + a * dt
            x = self.x0 + self.v0 * dt + 0.5 * a * dt**2
        elif t <= self.Ta + self.Tv:
            dt = t - self.Ta
            a = 0
            v = self.v_c
            x = self.x_acc_end + self.v_c * dt
        else:
            dt = t - self.Ta - self.Tv
            a = self.a_decel
            v = self.v_c + a * dt
            x = self.x_vel_end + self.v_c * dt + 0.5 * a * dt**2

        return (x, v, a)

# 示例数据测试
trap = Trapezoid()

# (0, trap.T, 0.0004)测试
# 两段加速
#trap.calculate_motion(-100, 100, 0, 0, 80, 15)
# 三段加速
#trap.calculate_motion(-100, 100, 0, 0, 10, 15)
# 恰好能加速到最大速度的场景
#trap.calculate_motion(-100, 100, 0, 0, 50, 12.5)
#trap.calculate_motion(100, -100, 0, 0, 50, 12.5)
# 指定了初始速度和终止速度的场景
#trap.calculate_motion(100, -100, -15, 20, 50, 60)
#trap.calculate_motion(x0=0, x1=100, v0=-5, v1=0, vmax=10, a=3)
#trap.calculate_motion(x0=0, x1=-200, v0=15, v1=5, vmax=20, a=4)
#trap.calculate_motion(x0=0, x1=500, v0=25, v1=0, vmax=30, a=1)
# 匀速场景
#trap.calculate_motion(-100, 100, 30, 30, 30, 15)
# 时间为负数
#trap.calculate_motion(0, 5, 10, 0, 15, 2)


# 改写为c语言时，绘图部分用打印plot_p的数据替代
# (0, trap.T, 1)测试
# max_speed  = max_rpm * 16384.0f / (60.0 * 2500.0);
# 1000 RPM = 
# 位置单位：编码器值
# 速度单位：每1/2500秒增加的编码器值
# 加速度单位：每1/2500秒变化的速度
trap.calculate_motion(0, 16384, 0, 0, 109, 3)

plot_p = []
plot_v = []
plot_a = []
for t in np.arange(0, trap.T, 1):
    x, v, a = trap.get_pos(t)
    plot_p.append(x)
    plot_v.append(v)
    plot_a.append(a)

print(plot_p)

t = np.arange(0, trap.T, 1)

plt.rcParams['font.sans-serif'] = ['STHeiti']
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t, plot_p)
plt.ylabel('位置')

plt.subplot(3, 1, 2)
plt.plot(t, plot_v)
plt.ylabel('速度')

plt.subplot(3, 1, 3)
plt.plot(t, plot_a)
plt.ylabel('加速度')

plt.show()