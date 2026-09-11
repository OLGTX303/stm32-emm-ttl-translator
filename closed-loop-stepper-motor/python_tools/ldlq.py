# 电机的电感是非线性的
# 考虑到最终的目的是使用电感Ld，Lq，磁链ψ参数估计Vd、Vq，采用电压方程的方式求解
# 根据永磁同步电机foc控制的电压方程，已知ω、Vd，Vq，Id，Iq，Rs的实测数据
# 求解Ld，Lq，ψ，使用python编程实现。
# d轴电压方程：
# Vd = Rs*Id − ω*Lq*Iq
# q轴电压方程：
# Vq = Rs*Iq + ω*Ld*Id + ω*ψ

# 对于Id较小（小于0.01A）的数据，可以认为Id=0，将方程简化为Vd = − ω*Lq*Iq， Vq = Rs*Iq + ω*ψ，仅参与Lq、ψ的计算
# 然后使用剩余的其他数据，计算Lq
# 应当选择适当的拟合算法，减少计算误差，最好能自动舍弃异常数据。

# 数据分组​​：根据Id的大小将数据分为两组。Id绝对值小于0.01A的用于计算Lq和ψ，其余的用于计算Ld。
# ​​异常值过滤​​：在计算Lq和ψ时，仅保留Vd和Iq符号相反的数据点，确保Lq为正。
# ​​鲁棒估计​​：使用中位数和IQR方法排除异常值，取均值作为最终结果。
import numpy as np
import re
# 给定参数
Rs = 2.0  # 欧姆

input_data = """ω=503.098rad/s, Vd=0.012V, Vq=2.495V, Id=-0.000A, Iq=0.063A, sqrt(Vd²+Vq²)=2.495
ω=766.949rad/s, Vd=0.016V, Vq=3.676V, Id=-0.000A, Iq=0.066A, sqrt(Vd²+Vq²)=3.676
ω=1030.497rad/s, Vd=-0.103V, Vq=4.860V, Id=0.000A, Iq=0.069A, sqrt(Vd²+Vq²)=4.861
ω=1294.694rad/s, Vd=-0.125V, Vq=6.021V, Id=-0.000A, Iq=0.073A, sqrt(Vd²+Vq²)=6.023
ω=1558.236rad/s, Vd=-0.371V, Vq=7.197V, Id=0.000A, Iq=0.076A, sqrt(Vd²+Vq²)=7.207
ω=1821.702rad/s, Vd=-0.556V, Vq=8.337V, Id=-0.000A, Iq=0.080A, sqrt(Vd²+Vq²)=8.356
ω=2084.851rad/s, Vd=-0.810V, Vq=9.491V, Id=0.000A, Iq=0.086A, sqrt(Vd²+Vq²)=9.526
ω=2348.581rad/s, Vd=-0.991V, Vq=10.626V, Id=-0.000A, Iq=0.091A, sqrt(Vd²+Vq²)=10.672
ω=2612.479rad/s, Vd=-1.389V, Vq=11.760V, Id=0.000A, Iq=0.095A, sqrt(Vd²+Vq²)=11.842
ω=2876.308rad/s, Vd=-1.621V, Vq=12.889V, Id=-0.000A, Iq=0.100A, sqrt(Vd²+Vq²)=12.991
ω=3139.949rad/s, Vd=-2.132V, Vq=13.980V, Id=-0.000A, Iq=0.105A, sqrt(Vd²+Vq²)=14.142
ω=3379.739rad/s, Vd=-2.458V, Vq=14.981V, Id=0.000A, Iq=0.110A, sqrt(Vd²+Vq²)=15.181
ω=3643.000rad/s, Vd=-2.865V, Vq=16.062V, Id=-0.000A, Iq=0.115A, sqrt(Vd²+Vq²)=16.315
ω=3906.378rad/s, Vd=-3.309V, Vq=17.148V, Id=-0.000A, Iq=0.121A, sqrt(Vd²+Vq²)=17.465
ω=4170.820rad/s, Vd=-3.784V, Vq=18.216V, Id=0.001A, Iq=0.125A, sqrt(Vd²+Vq²)=18.605
ω=4433.835rad/s, Vd=-4.391V, Vq=19.302V, Id=0.004A, Iq=0.130A, sqrt(Vd²+Vq²)=19.795
ω=4695.236rad/s, Vd=-4.997V, Vq=19.968V, Id=-0.012A, Iq=0.135A, sqrt(Vd²+Vq²)=20.584
ω=4963.155rad/s, Vd=-5.500V, Vq=20.230V, Id=-0.045A, Iq=0.143A, sqrt(Vd²+Vq²)=20.964
ω=5223.186rad/s, Vd=-6.016V, Vq=20.534V, Id=-0.071A, Iq=0.150A, sqrt(Vd²+Vq²)=21.397
ω=5485.949rad/s, Vd=-6.508V, Vq=20.621V, Id=-0.103A, Iq=0.157A, sqrt(Vd²+Vq²)=21.623
ω=5751.890rad/s, Vd=-6.926V, Vq=20.463V, Id=-0.139A, Iq=0.166A, sqrt(Vd²+Vq²)=21.603
ω=6014.519rad/s, Vd=-7.291V, Vq=20.079V, Id=-0.182A, Iq=0.175A, sqrt(Vd²+Vq²)=21.362
ω=6276.018rad/s, Vd=-7.613V, Vq=19.471V, Id=-0.226A, Iq=0.183A, sqrt(Vd²+Vq²)=20.907
ω=6545.646rad/s, Vd=-7.921V, Vq=18.980V, Id=-0.268A, Iq=0.196A, sqrt(Vd²+Vq²)=20.567
ω=6784.892rad/s, Vd=-8.213V, Vq=18.552V, Id=-0.301A, Iq=0.205A, sqrt(Vd²+Vq²)=20.289
ω=7043.266rad/s, Vd=-8.679V, Vq=18.372V, Id=-0.328A, Iq=0.209A, sqrt(Vd²+Vq²)=20.319
ω=7310.723rad/s, Vd=-8.616V, Vq=16.587V, Id=-0.394A, Iq=0.226A, sqrt(Vd²+Vq²)=18.691
ω=7581.889rad/s, Vd=-8.747V, Vq=15.808V, Id=-0.428A, Iq=0.234A, sqrt(Vd²+Vq²)=18.067
ω=7838.420rad/s, Vd=-8.959V, Vq=14.901V, Id=-0.470A, Iq=0.247A, sqrt(Vd²+Vq²)=17.387
ω=8100.043rad/s, Vd=-9.775V, Vq=16.375V, Id=-0.437A, Iq=0.249A, sqrt(Vd²+Vq²)=19.071
ω=8362.982rad/s, Vd=-10.037V, Vq=16.091V, Id=-0.453A, Iq=0.253A, sqrt(Vd²+Vq²)=18.965
ω=8621.258rad/s, Vd=-10.576V, Vq=16.357V, Id=-0.449A, Iq=0.262A, sqrt(Vd²+Vq²)=19.478
ω=8893.149rad/s, Vd=-10.742V, Vq=16.210V, Id=-0.462A, Iq=0.264A, sqrt(Vd²+Vq²)=19.446
ω=9122.822rad/s, Vd=-11.424V, Vq=16.636V, Id=-0.451A, Iq=0.274A, sqrt(Vd²+Vq²)=20.181
ω=9299.133rad/s, Vd=-11.487V, Vq=16.428V, Id=-0.458A, Iq=0.277A, sqrt(Vd²+Vq²)=20.045
ω=9452.798rad/s, Vd=-11.845V, Vq=16.712V, Id=-0.457A, Iq=0.279A, sqrt(Vd²+Vq²)=20.484
ω=9538.256rad/s, Vd=-11.941V, Vq=16.288V, Id=-0.465A, Iq=0.285A, sqrt(Vd²+Vq²)=20.196
ω=9469.592rad/s, Vd=-11.908V, Vq=16.543V, Id=-0.459A, Iq=0.283A, sqrt(Vd²+Vq²)=20.383
ω=9385.907rad/s, Vd=-11.847V, Vq=16.706V, Id=-0.454A, Iq=0.280A, sqrt(Vd²+Vq²)=20.480
"""
# 数据输入，格式：[ω, Vd, Vq, Id, Iq]
# 根据上文的字符串整理
# 解析输入数据到列表
data = []
for line in input_data.strip().split('\n'):
    # 初始化变量
    omega = vd = vq = id_ = iq = None
    # 分割键值对
    pairs = line.strip().split(', ')
    for pair in pairs:
        key, value_str = pair.split('=')
        # 使用正则表达式提取数值部分
        value = float(re.search(r'[-+]?\d+\.?\d*', value_str).group())
        if key == 'ω':
            omega = value
        elif key == 'Vd':
            vd = value
        elif key == 'Vq':
            vq = value
        elif key == 'Id':
            id_ = value
        elif key == 'Iq':
            iq = value
    # 确保所有值都已提取
    if None not in (omega, vd, vq, id_, iq):
        data.append([omega, vd, vq, id_, iq])

# 分离组1和组2
group1 = [point for point in data if abs(point[3]) < 0.01]
group2 = [point for point in data if abs(point[3]) >= 0.01]

# 处理组1，计算Lq和ψ
valid_group1 = [point for point in group1 if point[1] * point[4] < 0]

lq_values = []
psi_values = []

for point in valid_group1:
    omega, vd, vq, id_, iq = point
    # 计算Lq
    lq = -vd / (omega * iq)
    lq_values.append(lq)
    # 计算ψ
    psi = (vq - Rs * iq) / omega
    psi_values.append(psi)

# 异常值处理（Lq）
if lq_values:
    lq_median = np.median(lq_values)
    lq_q1 = np.percentile(lq_values, 25)
    lq_q3 = np.percentile(lq_values, 75)
    lq_iqr = lq_q3 - lq_q1
    lq_lower = lq_q1 - 1.5 * lq_iqr
    lq_upper = lq_q3 + 1.5 * lq_iqr
    filtered_lq = [lq for lq in lq_values if lq_lower <= lq <= lq_upper]
    final_lq = np.mean(filtered_lq)
else:
    final_lq = np.nan

# 异常值处理（ψ）
if psi_values:
    psi_median = np.median(psi_values)
    psi_q1 = np.percentile(psi_values, 25)
    psi_q3 = np.percentile(psi_values, 75)
    psi_iqr = psi_q3 - psi_q1
    psi_lower = psi_q1 - 1.5 * psi_iqr
    psi_upper = psi_q3 + 1.5 * psi_iqr
    filtered_psi = [psi for psi in psi_values if psi_lower <= psi <= psi_upper]
    final_psi = np.mean(filtered_psi)
else:
    final_psi = np.nan

# 处理组2，计算Ld
ld_values = []

for point in group2:
    omega, vd, vq, id_, iq = point
    if id_ == 0:
        continue  # 避免除以零，但根据分组条件id_绝对值>=0.01，不会出现
    numerator = vq - Rs * iq - omega * final_psi
    denominator = omega * id_
    ld = numerator / denominator
    ld_values.append(ld)

# 异常值处理（Ld）
if ld_values:
    ld_median = np.median(ld_values)
    ld_q1 = np.percentile(ld_values, 25)
    ld_q3 = np.percentile(ld_values, 75)
    ld_iqr = ld_q3 - ld_q1
    ld_lower = ld_q1 - 1.5 * ld_iqr
    ld_upper = ld_q3 + 1.5 * ld_iqr
    filtered_ld = [ld for ld in ld_values if ld_lower <= ld <= ld_upper]
    final_ld = np.mean(filtered_ld) if filtered_ld else np.nan
else:
    final_ld = np.nan

# 输出结果
print(f"计算得到的参数：")
print(f"Lq = {final_lq * 1000:.3f} mH")
print(f"ψ = {final_psi:.6f} Wb")
print(f"Ld = {final_ld * 1000:.3f} mH")

# =============== 验证与可视化部分 ===============
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

# 计算预测值
Vd_actual = []
Vq_actual = []
Vd_pred = []
Vq_pred = []

for point in data:
    omega, vd, vq, id_, iq = point
    # 使用估计出的参数计算预测值
    vd_p = Rs * id_ - omega * final_lq * iq
    vq_p = Rs * iq + omega * final_ld * id_ + omega * final_psi
    Vd_actual.append(vd)
    Vq_actual.append(vq)
    Vd_pred.append(vd_p)
    Vq_pred.append(vq_p)

# 计算误差指标
def print_metrics(name, actual, pred):
    mse = mean_squared_error(actual, pred)
    mae = mean_absolute_error(actual, pred)
    r2 = r2_score(actual, pred)
    print(f"{name}误差:")
    print(f"  MSE = {mse:.6f}")
    print(f"  MAE = {mae:.6f}")
    print(f"  R²  = {r2:.3f}")

print("\n验证结果:")
print_metrics("Vd", Vd_actual, Vd_pred)
print_metrics("Vq", Vq_actual, Vq_pred)

# 可视化对比
plt.figure(figsize=(14, 6))

# Vd对比
plt.subplot(1, 2, 1)
plt.scatter(Vd_actual, Vd_pred, c='blue', alpha=0.6, label='Pred vs Actual')
plt.plot([min(Vd_actual), max(Vd_actual)], [min(Vd_actual), max(Vd_actual)], 
         'r--', lw=2, label='Ideal Line')
plt.xlabel('Measured Vd (V)'), plt.ylabel('Predicted Vd (V)')
plt.title('Vd: Prediction vs Measurement')
plt.legend()

# Vq对比
plt.subplot(1, 2, 2)
plt.scatter(Vq_actual, Vq_pred, c='green', alpha=0.6, label='Pred vs Actual')
plt.plot([min(Vq_actual), max(Vq_actual)], [min(Vq_actual), max(Vq_actual)], 
         'r--', lw=2, label='Ideal Line')
plt.xlabel('Measured Vq (V)'), plt.ylabel('Predicted Vq (V)')
plt.title('Vq: Prediction vs Measurement')
plt.legend()

plt.tight_layout()
plt.show()
