import numpy as np
import matplotlib.pyplot as plt
# 绘制其他变量随ω变化的曲线图
# Vd Vq sqrt(Vd²+Vq²)在其中一张图里，Id Iq在另一张图里，两张图上下排列
input_data = """
ω=503.033rad/s, Vd=0.037V, Vq=2.481V, Id=-0.000A, Iq=0.062A, sqrt(Vd²+Vq²)=2.481
ω=766.985rad/s, Vd=0.054V, Vq=3.674V, Id=-0.000A, Iq=0.065A, sqrt(Vd²+Vq²)=3.674
ω=1031.193rad/s, Vd=-0.170V, Vq=4.863V, Id=-0.000A, Iq=0.068A, sqrt(Vd²+Vq²)=4.866
ω=1293.869rad/s, Vd=-0.160V, Vq=6.035V, Id=-0.000A, Iq=0.073A, sqrt(Vd²+Vq²)=6.037
ω=1558.646rad/s, Vd=-0.417V, Vq=7.212V, Id=0.000A, Iq=0.076A, sqrt(Vd²+Vq²)=7.224
ω=1822.006rad/s, Vd=-0.534V, Vq=8.366V, Id=-0.000A, Iq=0.080A, sqrt(Vd²+Vq²)=8.383
ω=2085.600rad/s, Vd=-0.767V, Vq=9.510V, Id=-0.000A, Iq=0.085A, sqrt(Vd²+Vq²)=9.541
ω=2349.054rad/s, Vd=-0.965V, Vq=10.649V, Id=0.000A, Iq=0.091A, sqrt(Vd²+Vq²)=10.693
ω=2612.667rad/s, Vd=-1.308V, Vq=11.789V, Id=0.000A, Iq=0.095A, sqrt(Vd²+Vq²)=11.862
ω=2875.927rad/s, Vd=-1.572V, Vq=12.904V, Id=0.000A, Iq=0.101A, sqrt(Vd²+Vq²)=12.999
ω=3139.785rad/s, Vd=-2.067V, Vq=14.000V, Id=-0.000A, Iq=0.106A, sqrt(Vd²+Vq²)=14.152
ω=3379.768rad/s, Vd=-2.454V, Vq=14.991V, Id=0.000A, Iq=0.110A, sqrt(Vd²+Vq²)=15.191
ω=3643.251rad/s, Vd=-2.785V, Vq=16.070V, Id=-0.000A, Iq=0.115A, sqrt(Vd²+Vq²)=16.309
ω=3906.161rad/s, Vd=-3.251V, Vq=17.154V, Id=0.000A, Iq=0.121A, sqrt(Vd²+Vq²)=17.459
ω=4170.973rad/s, Vd=-3.759V, Vq=18.287V, Id=0.004A, Iq=0.125A, sqrt(Vd²+Vq²)=18.669
ω=4430.880rad/s, Vd=-4.273V, Vq=19.432V, Id=0.010A, Iq=0.130A, sqrt(Vd²+Vq²)=19.896
ω=4700.362rad/s, Vd=-4.900V, Vq=19.681V, Id=-0.027A, Iq=0.135A, sqrt(Vd²+Vq²)=20.282
ω=4972.921rad/s, Vd=-5.387V, Vq=20.352V, Id=-0.041A, Iq=0.144A, sqrt(Vd²+Vq²)=21.053
ω=5213.951rad/s, Vd=-5.949V, Vq=20.256V, Id=-0.081A, Iq=0.152A, sqrt(Vd²+Vq²)=21.112
ω=5482.379rad/s, Vd=-6.418V, Vq=20.331V, Id=-0.115A, Iq=0.160A, sqrt(Vd²+Vq²)=21.320
ω=5752.609rad/s, Vd=-6.815V, Vq=20.277V, Id=-0.148A, Iq=0.169A, sqrt(Vd²+Vq²)=21.392
ω=6012.916rad/s, Vd=-7.234V, Vq=20.004V, Id=-0.184A, Iq=0.179A, sqrt(Vd²+Vq²)=21.272
ω=6277.446rad/s, Vd=-7.586V, Vq=19.375V, Id=-0.230A, Iq=0.188A, sqrt(Vd²+Vq²)=20.807
ω=6544.511rad/s, Vd=-7.706V, Vq=18.650V, Id=-0.279A, Iq=0.196A, sqrt(Vd²+Vq²)=20.179
ω=6784.137rad/s, Vd=-8.048V, Vq=18.038V, Id=-0.313A, Iq=0.209A, sqrt(Vd²+Vq²)=19.752
ω=7044.056rad/s, Vd=-8.520V, Vq=17.812V, Id=-0.344A, Iq=0.216A, sqrt(Vd²+Vq²)=19.745
ω=7311.208rad/s, Vd=-8.677V, Vq=17.430V, Id=-0.364A, Iq=0.223A, sqrt(Vd²+Vq²)=19.471
ω=7576.342rad/s, Vd=-9.052V, Vq=17.298V, Id=-0.384A, Iq=0.230A, sqrt(Vd²+Vq²)=19.523
ω=7840.217rad/s, Vd=-9.482V, Vq=17.469V, Id=-0.392A, Iq=0.237A, sqrt(Vd²+Vq²)=19.876
ω=8098.609rad/s, Vd=-9.880V, Vq=17.114V, Id=-0.411A, Iq=0.247A, sqrt(Vd²+Vq²)=19.761
ω=8364.978rad/s, Vd=-10.365V, Vq=17.164V, Id=-0.419A, Iq=0.255A, sqrt(Vd²+Vq²)=20.051
ω=8626.916rad/s, Vd=-10.407V, Vq=16.433V, Id=-0.446A, Iq=0.260A, sqrt(Vd²+Vq²)=19.451
ω=8888.023rad/s, Vd=-10.981V, Vq=16.685V, Id=-0.447A, Iq=0.268A, sqrt(Vd²+Vq²)=19.974
ω=9161.379rad/s, Vd=-11.109V, Vq=16.241V, Id=-0.465A, Iq=0.276A, sqrt(Vd²+Vq²)=19.676
ω=9419.338rad/s, Vd=-11.438V, Vq=15.412V, Id=-0.488A, Iq=0.290A, sqrt(Vd²+Vq²)=19.192
ω=9681.633rad/s, Vd=-11.593V, Vq=15.093V, Id=-0.502A, Iq=0.295A, sqrt(Vd²+Vq²)=19.031
ω=9911.171rad/s, Vd=-11.821V, Vq=14.647V, Id=-0.511A, Iq=0.302A, sqrt(Vd²+Vq²)=18.822
ω=10205.299rad/s, Vd=-12.618V, Vq=14.792V, Id=-0.508A, Iq=0.315A, sqrt(Vd²+Vq²)=19.443
ω=10132.879rad/s, Vd=-12.013V, Vq=15.363V, Id=-0.504A, Iq=0.296A, sqrt(Vd²+Vq²)=19.502
"""

# 提取数据
omegas, Vd, Vq, Id, Iq, sqrt_values = [], [], [], [], [], []
for line in input_data.strip().split('\n'):
    if not line.strip():
        continue
    parts = [p.split('=') for p in line.strip().split(', ')]
    data = {k: v for k, v in parts}
    omegas.append(float(data['ω'].replace('rad/s', '')))
    Vd.append(float(data['Vd'].replace('V', '')))
    Vq.append(float(data['Vq'].replace('V', '')))
    Id.append(float(data['Id'].replace('A', '')))
    Iq.append(float(data['Iq'].replace('A', '')))
    sqrt_values.append(float(data['sqrt(Vd²+Vq²)']))

# 创建上下排列的子图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# 定义单位转换函数
pole_pairs = 50  # 给定极对数

def rads_to_rpm(omega):
    """将电角速度(rad/s)转换为机械转速(RPM)"""
    return (omega * 60) / (2 * np.pi * pole_pairs)

def rpm_to_rads(rpm):
    """RPM转回rad/s（用于反向坐标轴）"""
    return (rpm * 2 * np.pi * pole_pairs) / 60

# 绘制电压相关曲线（上子图）
ax1.plot(omegas, Vd, label='Vd', marker='.', color='tab:blue')
ax1.plot(omegas, Vq, label='Vq', marker='^', color='tab:orange')
ax1.plot(omegas, sqrt_values, label='|V|', linestyle='--', color='tab:green')
ax1.set_title('Voltage Components vs Angular Frequency')
ax1.set_xlabel('Electrical Angular Velocity [rad/s]')
ax1.grid(True)
ax1.legend()

# 添加RPM次坐标轴（上子图）
ax1_sec = ax1.secondary_xaxis(
    'top', 
    functions=(rads_to_rpm, rpm_to_rads)
)
ax1_sec.set_xlabel('Mechanical Speed [RPM]')

# 绘制电流相关曲线（下子图）
ax2.plot(omegas, Id, label='Id', marker='x', color='tab:red')
ax2.plot(omegas, Iq, label='Iq', marker='x', color='tab:purple')
ax2.set_title('Current Components vs Angular Frequency')
ax2.set_xlabel('Electrical Angular Velocity [rad/s]')
ax2.grid(True)
ax2.legend()

# 添加RPM次坐标轴（下子图）
ax2_sec = ax2.secondary_xaxis(
    'top', 
    functions=(rads_to_rpm, rpm_to_rads)
)
ax2_sec.set_xlabel('Mechanical Speed [RPM]')

# 调整布局并显示
plt.tight_layout()
plt.show()