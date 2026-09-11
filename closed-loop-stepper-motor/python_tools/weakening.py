import math
import numpy as np
import matplotlib.pyplot as plt

# 电机参数配置
#E_peak = 6.0       # 反电动势峰值 (V)
#f_e = 211.1        # 反电动势频率 (Hz)
Vdc = 24.0         # 直流母线电压 (V)
eta = 0.8 * 2400 / 2500 # 电压利用率（%）
Ld = 6e-3        # d轴电感 (H)
Lq = 4e-3        # q轴电感 (H)
Ld_actual = 6e-3        # d轴电感 (H),分析设置电感与实际值偏差时的影响
Lq_actual = 4e-3        # q轴电感 (H)
P = 50             # 极对数
Rs = 2              # 内阻（Ω）
iq_target = 0.35    # 目标q轴电流 (A)

#psi = E_peak / (2 * math.pi * f_e)  # 计算磁链 (Wb)

psi = 0.004524 #0.004524
print(f"磁链: {psi:.6f} Wb")
# 电压限制计算
Vmax = Vdc * eta               # 最大相电压峰值

# 生成转速范围 (包含正负速)
w_rpm = np.linspace(-10, 4000, 2000)
w_e = w_rpm * (2 * math.pi * P) / 60  # 转换为电角速度 (rad/s)

# 初始化电流数组
iq_array = np.zeros_like(w_e)
id_array = np.zeros_like(w_e)
vq_array = np.zeros_like(w_e)
vd_array = np.zeros_like(w_e)
vdvq_array = np.zeros_like(w_e)
EPSILON = 1e-6  # 防止除零的极小量

for i, w in enumerate(w_e):
    if abs(w) < EPSILON:
        iq_array[i] = iq_target
        id_array[i] = 0
        continue
    
    w_abs = abs(w)
    max_weakening = 0.8 # 最大抵消永磁体磁场的80%，全部抵消会变得几乎没有转矩
    id_min = - psi * max_weakening / Ld
    # 输入参数是iq_target，如果能达到，尽量使id=iq，如果不能，维持iq尽量大
    # 根据电压约束(Ld *​ id​ + ψ)² + (Lq *​ iq​)² = (Vmax​ / ω​)²
    ld_id_psi = (Vmax / w_abs)**2 - (Lq * iq_target)**2
    if ((1 - max_weakening) * psi)**2 <= ld_id_psi:
        # 恒力矩
        iq = iq_target
        # (Ld *​ id​ + ψ)²
        id = ( math.sqrt(ld_id_psi) - psi ) / Ld
        if id > 0.5:
            id = 0.5
    else :
        Lqiq = (Vmax / w_abs)**2 - (Ld * id_min + psi)**2
        if Lqiq >= 0:
            iq = math.sqrt(Lqiq) / Lq
            if iq_target < 0:
                iq = -iq
        else:
            iq = 0 #ERROR
        id = id_min

    # 更新数组
    iq_array[i] = iq
    id_array[i] = id
    # d轴电压方程：
    # Vd = Rs*Id − ω*Lq*Iq
    # q轴电压方程：
    # Vq = Rs*Iq + ω*Ld*Id+ω*ψ
    vd_array[i] = Rs * id - w*Lq_actual*iq
    vq_array[i] = Rs * iq + w*Ld_actual*id + w * psi
    vdvq_array[i] = math.sqrt(vd_array[i]**2 + vq_array[i]**2)
    

# 绘制增强后的曲线
plt.rcParams['font.sans-serif'] = ['STHeiti']  # 显示中文
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7))

# 电流曲线
ax1.plot(w_rpm, iq_array, 'b-', lw=2, label='$I_q$ (A)')
ax1.plot(w_rpm, id_array, 'r--', lw=2, label='$I_d$ (A)')
ax1.legend()
ax2.plot(w_rpm, vq_array, 'y-', lw=2, label='$V_q$ (V)')
ax2.plot(w_rpm, vd_array, 'g--', lw=2, label='$V_d$ (V)')
ax2.plot(w_rpm, vdvq_array, 'b--', lw=2, label='$sqrt(vd²+vq²)$ (V)')
ax2.legend()
# 添加特征标注
plt.axvline(0, color='k', linestyle=':', alpha=0.5)
plt.axhline(0, color='k', linestyle=':', alpha=0.5)


plt.xlabel('机械转速 (RPM)')
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()