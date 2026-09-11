#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "hardware/flash.h"

#include "nr_micro_shell.h"
#include "foc.h"
#include "pin_def.h"

// 用于RS485通信
static volatile bool uart_rx = false;
static volatile absolute_time_t uart_rx_time;
static uint8_t uart_rx_buffer[256];
static int rs485_id = 0;
static uint8_t rs485_cmd_count = 0;

static void debug_print(int len, char *table_head)
{
    printf("START:%s\n",table_head);
    for(int i=0; i<len; i++){
        int *p = debug_data_buf[i];
        printf("%d,%d,%d,%d,%d\n",p[0],p[1],p[2],p[3],p[4]);
        sleep_ms(1);// 降低总线负载，减少对foc控制的影响
    }
    printf("STOP:\n");
}
static int angle_diff(int x, int y) {
    int diff = abs(x - y); // 计算直接差值
    if (diff > ENCODER_MAX / 2) { // 如果差值超过半圈
        diff = ENCODER_MAX - diff; // 取环形补数
    }
    return diff;
}
static int angle_average(int x, int y) {
    int diff = abs(x - y); // 计算直接差值
    int avg = (x + y) / 2; // 计算算术平均
    if (diff > ENCODER_MAX / 2) { // 如果差值超过半圈
        avg = (avg + ENCODER_MAX / 2) % ENCODER_MAX; // 调整平均方向
    }
    return avg;
}

static int calibration_mt6816(float test_voltage, int16_t *calibration)
{
    // 允许的正反转角度回差1.0°
    const int max_error = (int)(ENCODER_MAX * 1.0f / 360.0f);
    const int delay_time = 400;
    // 设置测试电压,对其初始角度
    foc_ctrl.mode = MODE_VOLATGE;
    foc_ctrl.vd = (int)(test_voltage / (VBUS_FACTOR * (float)foc_ctrl.power_voltage_filter) * (PWM_TOP + 1));
    printf("foc_ctrl.vd=%d\n", foc_ctrl.vd);
    foc_ctrl.vq = 0;
    foc_ctrl.electro_angle = 0;
    sleep_ms(100);
    // 使步进电机旋转到传感器零点附近最接近电角度零点的位置
    int prev = foc_ctrl.mt6816_angle_read;
    while(true){
        for(int i=0; i<1024; i++)
        {
            foc_ctrl.electro_angle = (foc_ctrl.electro_angle + 1) & 0x3FF;
            sleep_us(delay_time);
        }
        int now = foc_ctrl.mt6816_angle_read;
        if (now < prev){
            if(prev - now > ENCODER_MAX / 2){
                // 找到零点了
                printf("Found zero point\n");
                break;
            }else{
                // 存在线序问题
                printf("Reversing direction\n");
                foc_ctrl.reverse_v_alpha = !foc_ctrl.reverse_v_alpha;
                sleep_ms(100);
                prev = foc_ctrl.mt6816_angle_read;
            }
        }
        prev = now;
    }
    
    // 正转一圈，反转一圈，200步，把200个扇区的边界值采集起来，求平均值，消除回差
    // 正转
    foc_ctrl.electro_angle = 0;
    sleep_ms(100);
    for(int x=0; x<TOTAL_SECTOR; x++){
        int val = foc_ctrl.mt6816_angle_read;
        calibration[x] = val;
        printf("%d,%d\n", x, val);
        for(int i=0; i<256; i++){
            foc_ctrl.electro_angle = (foc_ctrl.electro_angle + 1) & 0x3FF;
            sleep_us(delay_time);
        }
    }
    sleep_ms(100);
    for(int i=0; i<256; i++){
        foc_ctrl.electro_angle = (foc_ctrl.electro_angle - 1) & 0x3FF;
        sleep_us(delay_time);
    }
    // 反转
    for(int x=TOTAL_SECTOR-1; x>=0; x--){
        //sleep_ms(50);
        int val = foc_ctrl.mt6816_angle_read;
        //printf("-calibration[%d]=%d,electro_angle=%d;\n", x, val,foc_ctrl.electro_angle);
        int diff = angle_diff(calibration[x], val);
        if(diff >= max_error){
            printf("ERROR: Calibration failed. angle_diff(%d, %d) >= %d.\n", calibration[x], val, max_error);
            return 1;
        }
        calibration[x] = angle_average(calibration[x], val);
        printf("%d,%d,%d\n", x, calibration[x],diff);
        for(int i=0; i<256; i++){
            foc_ctrl.electro_angle = (foc_ctrl.electro_angle - 1) & 0x3FF;
            sleep_us(delay_time);
        }
    }
    // 检查相邻扇区差值是否符合理论值
    const int expected_diff = (int)((float)ENCODER_MAX / TOTAL_SECTOR + 0.5f); // 四舍五入
    const int max_allowed_deviation = (int)(expected_diff * 0.25f); // 25%

    for (int i = 0; i < TOTAL_SECTOR; i++) {
        int current = calibration[i];
        int next = calibration[(i + 1) % TOTAL_SECTOR];
        int diff = angle_diff(current, next);
        if (abs(diff - expected_diff) > max_allowed_deviation) {
            printf("ERROR: Sector %d-%d difference %d exceeds 25%% of expected %d.\n",
                   i, (i + 1) % TOTAL_SECTOR, diff, expected_diff);
            return 1;
        }
    }

    // 处理跳变点，使数组递增
    int offset = 0;
    int previous = calibration[0];
    for (int i = 1; i < TOTAL_SECTOR; i++) {
        int current = calibration[i] + offset;
        if (current < previous) {
            int delta = previous - current;
            if (delta > ENCODER_MAX / 2) {
                offset += ENCODER_MAX;
                current += ENCODER_MAX;
            }
        }
        calibration[i] = current;
        previous = current;
    }

    // 调试打印：每20个数据一行
    printf("\nCalibration Data (adjusted):\n");
    for (int i = 0; i < TOTAL_SECTOR; i++) {
        printf("%5d", calibration[i]);
        if ((i + 1) % 20 == 0 || i == TOTAL_SECTOR - 1) {
            printf("\n");
        } else {
            printf(", ");
        }
    }
    // 计算线性拟合
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (int i = 0; i < TOTAL_SECTOR; i++) {
        sum_x += i;
        sum_y += calibration[i];
        sum_xy += i * calibration[i];
        sum_xx += i * i;
    }
    float slope = (TOTAL_SECTOR * sum_xy - sum_x * sum_y) / (TOTAL_SECTOR * sum_xx - sum_x * sum_x);
    float intercept = (sum_y - slope * sum_x) / TOTAL_SECTOR;

    // 计算最大非线性误差（相对满量程的百分比）
    float max_abs_error = 0; // 最大绝对误差
    float max_rel_error = 0; // 最大相对误差（非线性度）

    // 计算满量程范围（根据校准数据）
    int min_val = calibration[0];
    int max_val = calibration[0];
    for (int i = 0; i < TOTAL_SECTOR; i++) {
        if (calibration[i] < min_val) min_val = calibration[i];
        if (calibration[i] > max_val) max_val = calibration[i];
    }
    const float full_scale = max_val - min_val;

    for (int i = 0; i < TOTAL_SECTOR; i++) {
        const float expected = slope * i + intercept;
        const float error = calibration[i] - expected; // 误差值（带符号）
        
        // 更新最大绝对误差
        const float abs_error = fabsf(error);
        if (abs_error > max_abs_error) {
            max_abs_error = abs_error;
        }
        
        // 计算相对误差（非线性度百分比）
        const float rel_error = (abs_error / full_scale) * 100.0f;
        if (rel_error > max_rel_error) {
            max_rel_error = rel_error;
        }
    }
    foc_ctrl.vd = 0;
    foc_ctrl.vq = 0;
    printf("Non-linearity error:\n");
    printf("  Absolute: %.2f counts\n", max_abs_error);
    printf("  Relative: %.4f%%\n", max_rel_error);
    return 0;
}
// 电机型号17HS8401S-CZ
// 标称参数 步距角1.8°，额定电流1.8A，电阻1.8Ω，电感3.8mH，保持转矩52N·cm，转子惯量68g·cm²
// 实测参数：
// test_resistor_inductance测试：电阻2.0Ω(含MOS内阻0.1Ω)，电感4.1mH

// 估测绕组电感和电阻
static void test_resistor_inductance(float test_voltage, float *R, float *L)
{
    const int LEN = 2048;// <DEBUG_BUF_LEN
    int time[2] = {0,0};
    int current[2] = {0,0};
    int vd_test = (int)(test_voltage / (VBUS_FACTOR * (float)foc_ctrl.power_voltage_filter) * (PWM_TOP + 1));
    foc_ctrl.debug_div_5 = false;
    foc_ctrl.debug_date_address[0] = &foc_ctrl.i_alpha; //i_alpha
    foc_ctrl.debug_date_address[1] = &foc_ctrl.v_alpha;//v_alpha
    foc_ctrl.debug_date_address[2] = &foc_ctrl.i_beta; //i_beta
    foc_ctrl.debug_date_address[3] = &foc_ctrl.v_beta;//v_beta
    foc_ctrl.debug_date_address[4] = &foc_ctrl.zero;
    foc_ctrl.mode = MODE_VOLATGE;
    // 设定好电压，等待电流下降到36.8%的时间
    for (int i = 0; i < 2; i++)
    {
        foc_ctrl.vd = vd_test;
        foc_ctrl.vq = 0;
        foc_ctrl.electro_angle = 256 * i;
        sleep_ms(50);
        foc_ctrl.fill_debug_buf = LEN;
        sleep_ms(40);
        foc_ctrl.vd = 0;
        while(foc_ctrl.fill_debug_buf != 0){
            sleep_ms(1);
        }
        // debug_print(2048, "i_alpha,v_alpha,i_beta,v_beta,zero");

        // i=0时foc_ctrl.debug_date[k][0]和foc_ctrl.debug_date[k][1]有效
        // i=0时，需要计算foc_ctrl.debug_date[k][1] != 0 时foc_ctrl.debug_date[k][0]的平均值，写入current[0]
        // i=0时，需要计算从foc_ctrl.debug_date[k][1] == 0 开始，foc_ctrl.debug_date[k][0]从current[0]下降到
        // current[0] * (1/e)所需的时间（单位us，每个数据点40us），存入time[0]

        // 计算电流平均值和时间常数（优化后）
        int voltage_idx = (i == 0) ? 1 : 3;
        int current_idx = (i == 0) ? 0 : 2;
        // 计算稳定电流值
        long sum = 0;
        int count = 0;
        for(int k=0; k<LEN; k++){
            int* p = debug_data_buf[k];
            if(p[voltage_idx] != 0) { 
                sum += p[current_idx];
                count++;
            }
        }
        current[i] = (count > 0) ? sum / count : 0;
        // 寻找电压关闭点
        int k_start = -1;
        for(int k=0; k<LEN; k++){
            int* p = debug_data_buf[k];
            if(p[voltage_idx] == 0) {
                k_start = k;
                break;
            }
        }
        // 计算衰减时间
        time[i] = 0;
        if(k_start != -1 && current[i] > 0) {
            float target = current[i] * 0.3679f; // 1/e ≈ 0.3679
            for(int k=k_start; k<LEN-1; k++){
                int curr = debug_data_buf[k][current_idx];
                int next = debug_data_buf[k+1][current_idx];
                if(curr <= target) {
                    time[i] = (k - k_start) * 40;
                    break;
                } else if(curr > target && next <= target) {
                    float delta = (target - curr) / (float)(next - curr);
                    time[i] = (k - k_start + delta) * 40; 
                    break;
                }
            }
        }
        printf("current[%d]=%d time[%d]=%d\n",i,current[i],i,time[i]);
    }
    
    // 电流换算系数,量程范围-3.3A ~ 3.3A
    const float factor = 3.3f / 2048;
    // 电阻，单位Ω，含MOS内阻
    float resistor[2] = {test_voltage / (current[0] * factor), 
                         test_voltage / (current[1] * factor)}; 
    // 电感，单位mH
    float inductance[2] = {resistor[0] * time[0] / 1000.0f, 
                           resistor[1] * time[1] / 1000.0f};
    printf("power voltage = %.3fV\n"
           "resistor A = %.3f ohm, resistor B = %.3f ohm\n"
           "inductance A = %.3f mH, inductance B = %.3f mH\n",
           VBUS_FACTOR * foc_ctrl.power_voltage_filter, resistor[0], resistor[1], inductance[0], inductance[1]);
    *R = (resistor[1] + resistor[0]) / 2.0f;
    *L = (inductance[1] + inductance[0]) / 2.0f;
}
/*
static float sqrt_a2_b2(float a, float b)
{
    return sqrtf(a*a + b*b);
}
// 测量Ld、Lq
// （这个计算方式不合理，不再使用）
static void test_inductance_ld_lq(float test_voltage)
{
    const int LEN = 2048;// <DEBUG_BUF_LEN
    int vd_test = (int)(test_voltage / (VBUS_FACTOR * (float)foc_ctrl.power_voltage_filter) * (PWM_TOP + 1));
    foc_ctrl.debug_div_5 = false;
    foc_ctrl.debug_date_address[0] = &foc_ctrl.i_alpha; //i_alpha
    foc_ctrl.debug_date_address[1] = &foc_ctrl.v_alpha;//v_alpha
    foc_ctrl.debug_date_address[2] = &foc_ctrl.i_beta; //i_beta
    foc_ctrl.debug_date_address[3] = &foc_ctrl.v_beta;//v_beta
    foc_ctrl.debug_date_address[4] = &foc_ctrl.zero;
    foc_ctrl.mode = MODE_VOLATGE;
    const int num_of_test = 128;
    for (int i = 0; i < num_of_test; i++)
    {
        foc_ctrl.vd = vd_test;
        foc_ctrl.vq = 0;
        foc_ctrl.electro_angle = (1024 / num_of_test) * i;
        sleep_ms(50);
        foc_ctrl.fill_debug_buf = LEN;
        sleep_ms(40);
        foc_ctrl.vd = 0;
        while(foc_ctrl.fill_debug_buf != 0){
            sleep_ms(1);
        }
        // for debug
        //printf("-----i=%d------\n",i);
        // printf("START:\n");
        // for(int k=0; k<LEN; k++){
        //     int *p = debug_data_buf[k];
        //     printf("%d,%d,%d,%d\n",p[0],p[1],p[2],p[3]);
        // }
        // printf("STOP:\n");
        // 计算电流平均值和时间常数（优化后）
        const int voltage_idx_a = 1, voltage_idx_b = 3;
        const int current_idx_a = 0, current_idx_b = 2;
        // 计算稳定电流值
        float sum = 0;
        float current = 0;
        int count = 0;
        for(int k=0; k<LEN; k++){
            int* p = debug_data_buf[k];
            if(p[voltage_idx_a] != 0 || p[voltage_idx_b] != 0) { 
                sum += sqrt_a2_b2(p[current_idx_a], p[current_idx_b]);
                count++;
            }
        }
        current = (count > 0) ? sum / count : 0;
        // 寻找电压关闭点
        int k_start = -1;
        for(int k=0; k<LEN; k++){
            int* p = debug_data_buf[k];
            if(p[voltage_idx_a] == 0 && p[voltage_idx_b] == 0) {
                k_start = k;
                break;
            }
        }
        // 计算衰减时间
        int time = 0;
        if(k_start != -1 && current > 0) {
            float target = current * 0.3679f; // 1/e ≈ 0.3679
            for(int k=k_start; k<LEN-1; k++){
                float curr = sqrt_a2_b2(debug_data_buf[k][current_idx_a], debug_data_buf[k][current_idx_b]);
                float next = sqrt_a2_b2(debug_data_buf[k+1][current_idx_a], debug_data_buf[k+1][current_idx_b]);
                if(curr <= target) {
                    time = (k - k_start) * 40;
                    break;
                } else if(curr > target && next <= target) {
                    float delta = (target - curr) / (float)(next - curr);
                    time = (k - k_start + delta) * 40; 
                    break;
                }
            }
        }
        // printf("current=%f time=%d\n", current, time);
        const float factor = 3.3f / 2048; // 电流换算系数,量程范围-3.3A ~ 3.3A
        float resistor = test_voltage / (current * factor); // 电阻，单位Ω，含MOS内阻
        float inductance = resistor * time / 1000.0f; // 电感，单位mH
        printf("angle=%.1f, resistor = %.3f ohm, inductance = %.3f mH\n", 360.0f/num_of_test*i, resistor, inductance);
    }
}
*/

// 自动计算最佳的kp、ki参数的程序，要求动态响应好，允许有少量过冲
// resistor 绕组电阻，单位Ω
// inductance 绕组电感，单位mH
// voltage 电源电压
// design_bandwidth 环路带宽，单位Hz
// 例如电阻2.0Ω，电感4.1mH
void set_current_loop_kp_ki(float resistor, float inductance, float voltage, float design_bandwidth, int *kp, int *ki)
{
    // kp、ki缩放系数
    const int scale = 8192;
    // 实际输出电压 = output / voltage_pwm * voltage
    const int voltage_pwm = 2500;
    // 每秒钟计算PI控制的次数
    const int freq = 25000;
    // 电流换算系数，pi_ctrl的input或者target值，乘以current_factor，可得以A为单位的电流
    const float current_factor = 3.3f / 2048;
    // 物理量转换
    float L = inductance * 0.001f;    // mH转H
    float Ts = 1.0f / freq;           // 采样周期
    // 连续时间参数
    float Kp_cont = 2 * M_PI * L * design_bandwidth;
    float Ki_cont = resistor * 2 * M_PI * design_bandwidth;
    // 离散化转换
    float kp_physical = Kp_cont * current_factor * (voltage_pwm / voltage);
    float ki_physical = Ki_cont * current_factor * (voltage_pwm / voltage) * Ts;
    // 量化到scale
    *kp = (int)(kp_physical * scale + 0.5f);
    *ki = (int)(ki_physical * scale + 0.5f);

    //printf("Auto tuned: Kp=%d, Ki=%d, design_bandwidth=%.2fHz\n", *kp, *ki, design_bandwidth);
}
void shell_help_cmd(char argc, char *argv)
{
	unsigned int i = 0;
	for (i = 0; nr_shell.static_cmd[i].fp != NULL; i++)
	{
		shell_printf("%s, %s\n",nr_shell.static_cmd[i].cmd, nr_shell.static_cmd[i].description);
	}
}

void shell_reset_cmd(char argc, char *argv)
{
	watchdog_enable(10, 1);
}
void shell_prog_cmd(char argc, char *argv)
{
    reset_usb_boot(0,0);
}
// 修改为按照提前计算的位置序列进行运动控制
void shell_trap_cmd(char argc, char *argv)
{
    // 0xF9 111 110 01 current = 6 
    // 0xE1 111 000 01 current = 0
    static const uint8_t block_0[104] = {
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF9, 0xB6, 0xE2, 0x8A, 0x38, 0xA3, 0x8A, 0x46, 
        0xDD, 0x52, 0x37, 0x23, 0x91, 0xC7, 0x1C, 0x8E, 0x47, 0x1C, 0x72, 0x39, 0x1C, 0x71, 0xC8, 0xE4, 
        0x6E, 0xB7, 0x1C, 0x71, 0xC6, 0xDC, 0x8A, 0x36, 0xE3, 0x51, 0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 
        0xB7, 0x13, 0x71, 0x36, 0xE2, 0x6E, 0x26, 0xE2, 0x6D, 0xC4, 0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB8, 
        0x9B, 0x89, 0xB7, 0x13, 0x71, 0x36, 0xE2, 0x6E, 0x26, 0xE2, 0x6D, 0xC4, 0xDC, 0x4D, 0xC4, 0xDB, 
        0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13, 0x71, 0x37, 0x13, 0x6E, 0x26, 0xE2, 0x6D, 0xC4, 0xDC, 0x4D, 
        0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13,
    };
    static const uint8_t block_1[104] = {
        0x00, 0x01, 0x3B, 0x1F, 0x00, 0x00, 0x27, 0x00, 0xF9, 0x37, 0x13, 0x6E, 0x26, 0xE2, 0x6D, 0xC4, 
        0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13, 0x71, 0x37, 0x13, 0x6E, 0x26, 0xE2, 
        0x6E, 0x26, 0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13, 0x71, 0x37, 0x13, 0x6E, 
        0x26, 0xE2, 0x6E, 0x26, 0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB8, 0x9B, 0x89, 0xB7, 0x13, 0x71, 0x37, 
        0x13, 0x6E, 0x26, 0xE2, 0x6E, 0x26, 0xDC, 0x4D, 0xC4, 0xDB, 0x89, 0xB5, 0x14, 0x4A, 0x26, 0xD3, 
        0x69, 0xA6, 0xD2, 0x6D, 0x26, 0x93, 0x4D, 0x33, 0x0B, 0x69, 0x34, 0x9A, 0x69, 0xA4, 0xA1, 0x69, 
        0xA4, 0xD9, 0x6D, 0x34, 0xD3, 0x69, 0xA6, 0xDA, 
    };
    static const uint8_t block_2[13] = {
        0x0B, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0x01, 0x00, 0xF9, 0xB5, 0x13, 0x6D, 0x80, 
    };

    static const uint8_t *blocks[] = {block_0, block_1, block_2, 0};
    static const uint16_t block_size[] = {sizeof(block_0), sizeof(block_1), sizeof(block_2), 0};

    foc_ctrl.mode = MODE_POSITION;
    sleep_ms(1);
    foc_ctrl.multiple_angle_read = foc_ctrl.multiple_angle_read % ENCODER_MAX;
    foc_ctrl.multiple_angle_target = 0;
    sleep_ms(500);
    foc_ctrl.mode = MODE_POSITION_WITH_TRAPEZOID;
    foc_ctrl.debug_div_5 = true;
    foc_ctrl.debug_date_address[0] = &foc_ctrl.multiple_angle_target;
    foc_ctrl.debug_date_address[1] = &foc_ctrl.multiple_angle_read;
    foc_ctrl.debug_date_address[2] = &foc_ctrl.speed_target;
    foc_ctrl.debug_date_address[3] = &foc_ctrl.speed_read;
    foc_ctrl.debug_date_address[4] = &foc_ctrl.power_voltage_filter;
    foc_ctrl.fill_debug_buf = DEBUG_BUF_LEN;
    sleep_ms(10);

    // 写入初始数据
    int block_id = 0;
    decompress_and_write_fifo(blocks[block_id], block_size[block_id], &foc_fifo);
    block_id++;
    // p[4]的最高位，表示堵转时是否要暂停运动控制
    foc_ctrl.stop_when_block = false;
    foc_ctrl.max_error = 0;
    foc_ctrl.trap_run = true;
    // 追加数据
    while(true){
        uint32_t count = fifo_count(&foc_fifo);
        if(!foc_ctrl.trap_run){
            break;
        }else if(count < 256){
            if(blocks[block_id] != 0){
                decompress_and_write_fifo(blocks[block_id], block_size[block_id], &foc_fifo);
                block_id++;
            }
        }
        sleep_ms(1);
    }


    while(foc_ctrl.fill_debug_buf != 0){
        sleep_us(10);
    }

    debug_print(DEBUG_BUF_LEN,"angle_target,angle_read,speed_target,speed_read,power");
}
void shell_ff_cmd(char argc, char *argv)
{
    if(argc == 2){
        foc_ctrl.enable_ff = atoi(&(argv[(int)argv[1]]))==0 ? 0 : 1;
        printf("foc_ctrl.enable_ff = %d\n", foc_ctrl.enable_ff);
    }else{
        printf("ff 0/1\n");
    }
}
void shell_curr_pi_cmd(char argc, char *argv)
{
	float L, R, design_bandwidth = 1500.0f;
    int kp, ki;
    test_resistor_inductance(3.0f, &R, &L);
    printf("3.0V: L=%.3f mH, R=%.3f ohm\n", L, R);
    if(argc == 2){
        design_bandwidth = atoff(&(argv[(int)argv[1]]));
    }
    // set_current_loop_kp_ki(R, L, VBUS_FACTOR * foc_ctrl.power_voltage_filter, 1000.0f, &kp, &ki);
    // printf("Suggest kp = %d, ki = %d\n", kp, ki);
    set_current_loop_kp_ki(R, L, VBUS_FACTOR, design_bandwidth, &kp, &ki);
    printf("design_bandwidth = %.2f, Suggest kp = %d, ki = %d\n", design_bandwidth, kp, ki);
    foc_ctrl.curr_kp = kp;
    foc_ctrl.curr_ki = ki;
    foc_ctrl.Ld = 0.001f * L;
    foc_ctrl.Lq = 0.001f * L;
    foc_ctrl.Rs = R;
}
void shell_calibration_cmd(char argc, char *argv)
{
    printf("Start calibration, please wait...\n");
	calibration_mt6816(3.0f, calibration);
    calibration[TOTAL_SECTOR] = calibration[0] + ENCODER_MAX; 
}
void shell_psi_cmd(char argc, char *argv)
{
    int max_speed_in_rpm = 500;
    foc_ctrl.low_speed_id_current = 0.0f;
    if(argc == 2){
        max_speed_in_rpm = atoi(&(argv[(int)argv[1]]));
    }
    if(max_speed_in_rpm < 200){
        max_speed_in_rpm = 200;
    }else if(max_speed_in_rpm > 5000){
        max_speed_in_rpm = 5000;
    }
    printf("max_speed_in_rpm = %d\n", max_speed_in_rpm);
    float psi_sum = 0;
    int psi_count = 0;
    foc_ctrl.mode = MODE_SPEED;
    foc_ctrl.speed_target = 0;
    for(int i=100; i<=max_speed_in_rpm; i+=50){
        foc_ctrl.speed_target = i * RPM_TO_SPEED;
        // 等待转速稳定，然后开始测量
        sleep_ms(300);
        foc_ctrl.debug_div_5 = false;
        foc_ctrl.debug_date_address[0] = &foc_ctrl.vd;
        foc_ctrl.debug_date_address[1] = &foc_ctrl.vq;
        foc_ctrl.debug_date_address[2] = &foc_ctrl.id;
        foc_ctrl.debug_date_address[3] = &foc_ctrl.iq;
        foc_ctrl.debug_date_address[4] = &foc_ctrl.speed_read;
        foc_ctrl.fill_debug_buf = DEBUG_BUF_LEN;
        while(foc_ctrl.fill_debug_buf != 0){
            sleep_us(10);
        }
        // 计算平均值
        int vd_sum=0, vq_sum=0, id_sum=0, iq_sum=0, speed_sum=0;
        for(int k=0; k<DEBUG_BUF_LEN; k++){
            int* p = debug_data_buf[k];
            vd_sum += p[0];
            vq_sum += p[1];
            id_sum += p[2];
            iq_sum += p[3];
            speed_sum += p[4];
        }
        
        // 转换为标准单位
        float power_voltage = VBUS_FACTOR * foc_ctrl.power_voltage_filter; // 单位V
        float vd = power_voltage * vd_sum / ((PWM_TOP + 1) * DEBUG_BUF_LEN);
        float vq = power_voltage * vq_sum / ((PWM_TOP + 1) * DEBUG_BUF_LEN);
        float id = id_sum * 3.3f / 2048.0f / DEBUG_BUF_LEN; 
        float iq = iq_sum * 3.3f / 2048.0f / DEBUG_BUF_LEN; 
        float w = speed_sum * SPEED_TO_W / DEBUG_BUF_LEN;
        float sqrt_vd2_vq2 = sqrtf(vd*vd + vq*vq);
        printf("ω=%.3frad/s, Vd=%.3fV, Vq=%.3fV, Id=%.3fA, Iq=%.3fA, sqrt(Vd²+Vq²)=%.3f\n",w,vd,vq,id,iq,sqrt_vd2_vq2);
        // d轴电压方程：
        // Vd = Rs*Id − ω*Lq*Iq
        // q轴电压方程：
        // Vq = Rs*Iq + ω*Ld*Id+ω*ψ
        if(fabsf(id) < 0.01f){
            float psi = (vq - foc_ctrl.Rs * iq) / w;
            psi_sum += psi;
            psi_count ++;
            // printf("psi=%f\n",psi);
        }
    }
    foc_ctrl.speed_target = 0;
    psi_sum = psi_sum / psi_count;
    printf("set foc_ctrl.psi = %f\n", psi_sum);
    foc_ctrl.psi = psi_sum;
    foc_ctrl.low_speed_id_current = LOW_SPEED_CURRENT;
}
void shell_485_cmd(char argc, char *argv)
{
    gpio_put(UART_485_EN, 1);
    uart_puts(UART_ID, "Hello, RS485!\n");
    uart_tx_wait_blocking(UART_ID);
    gpio_put(UART_485_EN, 0);
    // uint8_t rx;
    // int cnt = 0;
    // while(++cnt <= 20){
    //     uart_read_blocking(UART_ID, &rx, 1);
    //     printf("%c read.\n",rx);
    // }
}
// 捕捉调试用的波形
void shell_speed_cmd(char argc, char *argv)
{
    char *info;
    if(argc == 3)
	{
        foc_ctrl.speed_kp = atoi(&(argv[(int)argv[1]]));
        printf("foc_ctrl.speed_kp=%d\n", foc_ctrl.speed_kp);
        foc_ctrl.speed_ki = atoi(&(argv[(int)argv[2]]));
        printf("foc_ctrl.speed_ki=%d\n", foc_ctrl.speed_ki);
    }
    int max_speed_in_rpm = 500;
    if(argc == 2)
	{
        max_speed_in_rpm = atoi(&(argv[(int)argv[1]]));
        foc_ctrl.debug_div_5 = false;
        foc_ctrl.debug_date_address[0] = &foc_ctrl.id;
        foc_ctrl.debug_date_address[1] = &foc_ctrl.id_target;
        foc_ctrl.debug_date_address[2] = &foc_ctrl.iq;
        foc_ctrl.debug_date_address[3] = &foc_ctrl.iq_target;
        foc_ctrl.debug_date_address[4] = &foc_ctrl.speed_read;
        info = "id,id_target,iq,iq_target,speed";
    }else{
        foc_ctrl.debug_div_5 = false;
        foc_ctrl.debug_date_address[0] = &foc_ctrl.speed_target;
        foc_ctrl.debug_date_address[1] = &foc_ctrl.speed_read;
        foc_ctrl.debug_date_address[2] = &foc_ctrl.id_target;
        foc_ctrl.debug_date_address[3] = &foc_ctrl.iq_target;
        foc_ctrl.debug_date_address[4] = &foc_ctrl.power_voltage_filter;
        info = "speed_target,speed_read,id_target,iq_target,power_voltage";
    }

    foc_ctrl.speed_target = 0;
    foc_ctrl.mode = MODE_SPEED;
    sleep_ms(300);
    foc_ctrl.fill_debug_buf = DEBUG_BUF_LEN;
    sleep_ms(1);
    foc_ctrl.speed_target = max_speed_in_rpm * RPM_TO_SPEED;
    sleep_ms(100);
    foc_ctrl.speed_target = 0;
    while(foc_ctrl.fill_debug_buf != 0){
        sleep_us(10);
    }
    debug_print(DEBUG_BUF_LEN, info);
}
void shell_pos_cmd(char argc, char *argv)
{
    if(argc == 3)
	{
        foc_ctrl.pos_kp = atoi(&(argv[(int)argv[1]]));
        printf("foc_ctrl.pos_kp=%d\n", foc_ctrl.pos_kp);
        foc_ctrl.pos_kd = atoi(&(argv[(int)argv[2]]));
        printf("foc_ctrl.pos_kd=%d\n", foc_ctrl.pos_kd);
    }
    int pos = 16384;
    if(argc == 2)
	{
        pos = atoi(&(argv[(int)argv[1]]));
    }
    foc_ctrl.multiple_angle_read = foc_ctrl.multiple_angle_read % ENCODER_MAX;
    foc_ctrl.multiple_angle_target = 0;
    foc_ctrl.mode = MODE_POSITION;
    sleep_ms(300);
    foc_ctrl.debug_div_5 = true;
    foc_ctrl.debug_date_address[0] = &foc_ctrl.multiple_angle_target;
    foc_ctrl.debug_date_address[1] = &foc_ctrl.multiple_angle_read;
    foc_ctrl.debug_date_address[2] = &foc_ctrl.speed_target;
    foc_ctrl.debug_date_address[3] = &foc_ctrl.speed_read;
    foc_ctrl.debug_date_address[4] = &foc_ctrl.power_voltage_filter;
    foc_ctrl.fill_debug_buf = DEBUG_BUF_LEN;
    sleep_ms(1);
    foc_ctrl.multiple_angle_target += pos;
    sleep_ms(300);
    foc_ctrl.multiple_angle_target -= pos;
    
    while(foc_ctrl.fill_debug_buf != 0){
        sleep_us(10);
    }

    debug_print(DEBUG_BUF_LEN,"angle_target,angle_read,speed_target,speed_read,power_voltage");
}
void shell_curr_cmd(char argc, char *argv)
{
    if(argc == 3)
	{
	    float iq = atoff(&(argv[(int)argv[1]]));
        float id = atoff(&(argv[(int)argv[2]]));
        int iiq = roundf(iq / 3.3f * 2048);
        int iid = roundf(id / 3.3f * 2048);
        if(abs(iiq) > MAX_CURRENT || abs(iid) > MAX_CURRENT){
            printf("too large.\n");
        }else{
            foc_ctrl.mode = MODE_CURRENT;
            foc_ctrl.iq_target = 0;
            foc_ctrl.id_target = 0;
            sleep_ms(100);
            foc_ctrl.debug_div_5 = false;
            foc_ctrl.debug_date_address[0] = &foc_ctrl.id;
            foc_ctrl.debug_date_address[1] = &foc_ctrl.iq;
            foc_ctrl.debug_date_address[2] = &foc_ctrl.i_alpha;
            foc_ctrl.debug_date_address[3] = &foc_ctrl.i_beta;
            foc_ctrl.debug_date_address[4] = &foc_ctrl.power_voltage_filter;
            foc_ctrl.fill_debug_buf = DEBUG_BUF_LEN;
            sleep_ms(1);
            foc_ctrl.iq_target = iiq;
            foc_ctrl.id_target = iid;
            sleep_ms(100);
            foc_ctrl.iq_target = 0;
            foc_ctrl.id_target = 0;
            while(foc_ctrl.fill_debug_buf != 0){
                sleep_us(10);
            }

            debug_print(DEBUG_BUF_LEN,"id,iq,i_alpha,i_beta,power_voltage");
        }
    }else{
        printf("Usage: curr iq id\n");
    }
}

float read_onboard_temperature(int temperature_raw) 
{
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);
    float adc = (float)temperature_raw * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    return tempC;
}
// 输出驱动板的状态
void shell_stat_cmd(char argc, char *argv)
{
    float temp = read_onboard_temperature(foc_ctrl.temperature_raw);
    printf("temperature = %.1f\n", temp);
    printf("power voltage = %.1fV\n", VBUS_FACTOR * foc_ctrl.power_voltage_raw);
    printf("mode = %d\n", foc_ctrl.mode);
    printf("multiple_angle_read = %d\n", foc_ctrl.multiple_angle_read);
    printf("rs485_cmd_count = %d\n", (int)rs485_cmd_count);
    
}
// --------------------------- 参数存储与调试 ----------------------------------
// CRC8计算
static uint8_t crc8(uint8_t *datagram, int datagramLength)
{
    int i, j;
    uint8_t currentByte;
    uint8_t crc = 0;
    for (i = 0; i < datagramLength; i++)
    {                              // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    }
    return crc;
}

typedef enum {
    PARAM_INT = 0,
    PARAM_FLOAT = 1
}param_type;

typedef struct{
    const char *name;
    volatile void *data;
    param_type type;
} param;

#define NUM_PARAMS 12
param params[NUM_PARAMS] = {
    {"pos_kp", &foc_ctrl.pos_kp, PARAM_INT},
    {"pos_kd", &foc_ctrl.pos_kd, PARAM_INT},
    {"speed_kp", &foc_ctrl.speed_kp, PARAM_INT},
    {"speed_ki", &foc_ctrl.speed_ki, PARAM_INT},
    {"curr_kp", &foc_ctrl.curr_kp, PARAM_INT},
    {"curr_ki", &foc_ctrl.curr_ki, PARAM_INT},
    {"psi", &foc_ctrl.psi, PARAM_FLOAT},
    {"Ld", &foc_ctrl.Ld, PARAM_FLOAT},
    {"Lq", &foc_ctrl.Lq, PARAM_FLOAT},
    {"reverse_v_alpha", &foc_ctrl.reverse_v_alpha, PARAM_INT},
    {"Rs", &foc_ctrl.Rs, PARAM_FLOAT},
    {"rs485_id", &rs485_id, PARAM_INT},
};

// FLASH总容量2MB，参数存储在偏移量为512KB的位置，存储位置的偏移量，必须是4096的整数倍
#define FLASH_TARGET_OFFSET (512 * 1024)
#define FLASH_CONFIG_VERSION (0x01)
// flash_config，大小是512字节，手工对齐，防止编译器版本影响数据排列
#pragma pack(push, 1)  // 强制 1 字节对齐，取消填充
typedef struct{
    int32_t pos_kp;
    int32_t pos_kd;
    int32_t speed_kp;
    int32_t speed_ki;
    int32_t curr_kp;
    int32_t curr_ki;
    float psi;
    float Ld;
    float Lq;
    int32_t reverse_v_alpha;
    float Rs;
    int32_t rsv_32bit[16];
    int16_t calibration[200];
    uint8_t version;
    uint8_t rs485_id;
    uint8_t rsv_8bit;
    uint8_t crc8;// 偏移量511
} flash_config; 
#pragma pack(pop)
static_assert(sizeof(flash_config) == 512, "flash_config size error");
static_assert(offsetof(flash_config, crc8) == 511, "crc8 offset error");

static void shell_save_cmd(char argc, char *argv)
{
    printf("stop motor\n");
    foc_ctrl.mode = MODE_VOLATGE;
    foc_ctrl.vd = 0;
    foc_ctrl.vq = 0;
    sleep_ms(100);
    multicore_reset_core1(); // put core 1 into a deep sleep.
    
    flash_config cfg;
    memset(&cfg, 0, sizeof(flash_config));
    cfg.version = FLASH_CONFIG_VERSION;
    cfg.pos_kp = foc_ctrl.pos_kp;
    cfg.pos_kd = foc_ctrl.pos_kd;
    cfg.speed_kp = foc_ctrl.speed_kp;
    cfg.speed_ki = foc_ctrl.speed_ki;
    cfg.curr_kp = foc_ctrl.curr_kp;
    cfg.curr_ki = foc_ctrl.curr_ki;
    cfg.psi = foc_ctrl.psi;
    cfg.Ld = foc_ctrl.Ld;
    cfg.Lq = foc_ctrl.Lq;
    cfg.reverse_v_alpha = foc_ctrl.reverse_v_alpha;
    cfg.Rs = foc_ctrl.Rs;
    cfg.rs485_id = rs485_id;
    memcpy(cfg.calibration, calibration, 200*sizeof(int16_t));
    cfg.crc8 = crc8((uint8_t *)&cfg, 511);

    // Note that a whole number of sectors must be erased at a time.
    // FLASH_SECTOR_SIZE = 4KB
    // FLASH_PAGE_SIZE = 256B
    printf("write flash, version=0x%x, crc=0x%x\n", cfg.version, cfg.crc8);

    printf("erase\n");
    uint32_t save = save_and_disable_interrupts(); // 关闭中断，如果中间出现中断，flash操作可能会卡死
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(save);

    printf("program\n");
    save = save_and_disable_interrupts();
    flash_range_program(FLASH_TARGET_OFFSET, (uint8_t *)&cfg, FLASH_PAGE_SIZE * 2);
    restore_interrupts(save);

    printf("verify\n");
    const flash_config *cfg_in_flash = (const flash_config *) (XIP_BASE + FLASH_TARGET_OFFSET);
    if(memcmp(cfg_in_flash, &cfg, sizeof(flash_config)) == 0){
        printf("verify success\n");
    }else{
        printf("verify fail\n");
    }
    // 有个bug，保存参数可能会造成USB通信断开
    // 采取直接复位单片机的规避措施
    printf("reset mcu\n");
    watchdog_enable(10, 1);
}

static void shell_load_cmd(char argc, char *argv)
{
    // 这个地址位于FLASH，是只读的
    const flash_config *cfg = (const flash_config *) (XIP_BASE + FLASH_TARGET_OFFSET);
    // 判断有效性
    uint8_t crc = crc8((uint8_t *)cfg, 511);
    if(crc == cfg->crc8 && FLASH_CONFIG_VERSION == cfg->version){
        foc_ctrl.pos_kp = cfg->pos_kp;
        foc_ctrl.pos_kd = cfg->pos_kd;
        foc_ctrl.speed_kp = cfg->speed_kp;
        foc_ctrl.speed_ki = cfg->speed_ki;
        foc_ctrl.curr_kp = cfg->curr_kp;
        foc_ctrl.curr_ki = cfg->curr_ki;
        foc_ctrl.psi = cfg->psi;
        foc_ctrl.Ld = cfg->Ld;
        foc_ctrl.Lq = cfg->Lq;
        foc_ctrl.reverse_v_alpha = cfg->reverse_v_alpha;
        foc_ctrl.Rs = cfg->Rs;
        rs485_id = cfg->rs485_id;
        memcpy(calibration, cfg->calibration, 200*sizeof(int16_t));
        calibration[TOTAL_SECTOR] = calibration[0] + ENCODER_MAX; 
        printf("read flash, version=0x%x, crc=0x%x\n", cfg->version, cfg->crc8);
    }else{
        printf("invalid data in flash, version=0x%x, crc=0x%x\n", cfg->version, cfg->crc8);
    }
}

// 输出全部可配置参数
void shell_dump_cmd(char argc, char *argv)
{
    // 根据params，输出每个参数的值
    printf("Params:\n");
    for (int i = 0; i < NUM_PARAMS; i++) {
        if (params[i].type == PARAM_INT) {
            printf("%-16s: %d\n", params[i].name, *(volatile int*)params[i].data);
        } else if(params[i].type == PARAM_FLOAT ){
            printf("%-16s: %f\n", params[i].name, *(volatile float*)params[i].data);
        }
    }
    printf("\nCalibration Data:\n");
    for (int i = 0; i < TOTAL_SECTOR; i++) {
        printf("%5d", calibration[i]);
        if ((i + 1) % 20 == 0 || i == TOTAL_SECTOR - 1) {
            printf("\n");
        } else {
            printf(", ");
        }
    }
}
// 设置可配置参数
void shell_set_cmd(char argc, char *argv)
{
    if (argc != 3) {
        printf("usage: set param value\n");
        return;
    }
    // 例如 name = "psi" value = "0.004463"
    // 例如 name = "speed_kp" value = "90000"
    char *name = &(argv[(int)argv[1]]);
    char *value_str = &(argv[(int)argv[2]]);

    int param_found = 0;

    for (int i = 0; i < NUM_PARAMS; i++) {
        if (strcmp(params[i].name, name) == 0) {
            param_found = 1;
            if (params[i].type == PARAM_INT) {
                char *endptr;
                long int value = strtol(value_str, &endptr, 10);
                if (*endptr != '\0') {
                    printf("Error: invalid int '%s'\n", value_str);
                    return;
                }
                *(volatile int *)params[i].data = (int)value;
                printf("set %s = %d\n", name, (int)value);
            } else {
                char *endptr;
                float value = strtof(value_str, &endptr);
                if (*endptr != '\0') {
                    printf("Error: invalid float '%s'\n", value_str);
                    return;
                }
                *(volatile float *)params[i].data = value;
                printf("set %s = %f\n", name, value);
            }
            break;
        }
    }

    if (!param_found) {
        printf("Error: unknown param '%s'\n", name);
    }
}


const static_cmd_st static_cmd[] = {
    {"help", shell_help_cmd, "Display available commands and descriptions"},
    {"prog", shell_prog_cmd, "Reset MCU and enter USB bootloader mode"},
    {"reset", shell_reset_cmd, "Perform hardware reset of microcontroller"},
    {"trap", shell_trap_cmd, "Test trapezoidal velocity profile control"},
    {"curr_pi", shell_curr_pi_cmd, "Auto-tune current loop PI parameters based on bandwidth"},
    {"psi", shell_psi_cmd, "Measure motor flux linkage (ψ) parameter"},
    {"calibration", shell_calibration_cmd, "Calibrate MT6816 magnetic encoder alignment"},
    {"485", shell_485_cmd, "RS485 communication interface test"},
    {"speed", shell_speed_cmd, "Test speed loop control with data capture"},
    {"curr", shell_curr_cmd, "Test current loop control with specified dq-axis currents"},
    {"pos", shell_pos_cmd, "Test position loop control with PID parameters"},
    {"ff", shell_ff_cmd, "Feedforward control toggle (0: disable, 1: enable)"},
    {"stat", shell_stat_cmd, "Show system status (temp, voltage, encoder, etc.)"},
    {"dump", shell_dump_cmd, "Display all configurable parameters and calibration data"},
    {"set", shell_set_cmd, "Modify configuration parameters (set <param> <value>)"},
    {"save", shell_save_cmd, "Save parameters to non-volatile flash memory"},
    {"load", shell_load_cmd, "Load parameters from flash memory"},
    {"\0", NULL, NULL}
};


// 串口RX中断，根据FF FF包头同步，并且接收数据
// 例如 ff ff 07 01 01 00 4f 
// 不管总线上的数据是否是需要的，都会进行接收，以免丢失同步信号
// 例如上一个数据包以FF结尾，下一个数据包FF FF开头，会出现同步异常
void on_uart_rx() 
{
    const int WAIT_FIRST_FF = 0;
    const int WAIT_SECOND_FF = 1;
    const int WAIT_LENGTH = 2;
    uint8_t *p = uart_rx_buffer;

    static int stat = WAIT_FIRST_FF;
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        if(stat == WAIT_FIRST_FF){
            if(ch == 0xFF){
                stat = WAIT_SECOND_FF;
            }else{
                stat = WAIT_FIRST_FF;
            }
        }else if(stat == WAIT_SECOND_FF){
            if(ch == 0xFF){
                stat = WAIT_LENGTH;
            }else{
                stat = WAIT_FIRST_FF;
            }
        }else if(stat == WAIT_LENGTH){
            if(ch >= 5 && ch <= 128){
                p[0] = 0xff;
                p[1] = 0xff;
                p[2] = ch;
                stat++;
            }else{
                stat = WAIT_FIRST_FF;
            }
        }else if(stat > WAIT_LENGTH){
            if(stat < p[2] - 1){
                p[stat] = ch;
                stat++;
            }else if(stat == p[2] - 1){
                p[stat] = ch; // CRC
                uart_rx = true;
                uart_rx_time = get_absolute_time();
                stat = WAIT_FIRST_FF;
            }
        }
    }
}
/*
使用c语言设计一个一套FIFO
深度为512，存放int32_t数据
在RP2040使用，需要多核心访问，使用spin_lock_blocking、spin_unlock同步
需要一个读取函数，一次从FIFO取出一个数据
需要一个写入函数，一次向FIFO写入多个数据，数据数量可变
需要一个获取当前FIFO中数据数量的函数
*/


/*
    ===== 主机→电机控制器数据帧格式 =====
    [字头] + [数据长度] + [指令类型 + ID号] + [数据] + [CRC8]

    [01字头]             uint16_t 固定值0xFFFF（2字节）
    [2 数据长度]          uint8_t len（1字节），指数据包的总长度，含0xFFFF和校验和
    [3 指令类型 + ID号]   低4bit：取值范围1-4，0用于广播
                        高4bit：指令类型
    [n-1 数据]           uint8_t data[250] 最大250字节
    [n CRC8]            uint8_t 校验值（含字头）

    注：每个电机仅处理ID匹配的指令块，其他忽略
*/
typedef enum{
    RS485_CMD_STAT = 0,
    RS485_CMD_ENABLE = 1,
    RS485_CMD_DISABLE = 2,
    RS485_CMD_RESET = 3,
    RS485_CMD_CLEAR_FIFO = 4,
    RS485_CMD_SYNC = 5,
    RS485_CMD_FIFO = 6,
}rs485_cmd;
int decode_uart_rx_date(uint8_t *p)
{
    const int RX_DELAY = 50; // us
    // 是能梯形加速控制后，第一次运行trap指令
    // static bool trap_first_run = true;
    bool need_ack = false;

    int pkg_length = p[2];
    uint8_t crc = crc8(p, pkg_length - 1);

    // for(int i=0; i<pkg_length; i++){
    //     printf("%02x ", p[i]);
    // }
    // printf("\n");

    if(p[pkg_length - 1] != crc){
        printf("485: crc error\n");
        return 1;
    }
    if(p[3] == 0xff){
        // 这个包是其他控制器发给主机的
        return 2;
    }
    uint8_t id = p[3] & 0x0f;
    if(id != rs485_id && id != 0){
        // 这个包是发给其他控制器的
        return 3;
    }
    rs485_cmd cmd = p[3] >> 4;
    // printf("id=%d, cmd=%d\n", id, cmd);
    if(cmd == RS485_CMD_STAT && pkg_length == 5){
        // ------------------ 状态查询指令 RS485_CMD_STAT -----------------------
        rs485_cmd_count ++;
        // 响应数据结构（固定12字节）：
        // 历史接收指令计数,超过255会回到0
        uint8_t cmd_count = rs485_cmd_count; 
        // 上一次查询到现在的最大控制误差
        uint16_t max_error = foc_ctrl.max_error;
        foc_ctrl.max_error = 0;
        // 运动控制器状态（高1bit）+ 堵转状态（次高1bit）+ FIFO已用空间（低14bit）
        uint16_t fifo_status = foc_ctrl.trap_run ? 0x8000 : 0;
        fifo_status |= foc_ctrl.block ? 0x4000 : 0;
        fifo_status |= fifo_count(&foc_fifo) & 0x3FFF;
        // 温度值（℃）
        int8_t temperature = roundf( read_onboard_temperature(foc_ctrl.temperature_raw) ); 
        // 当前位置（小端序）
        int32_t pos = foc_ctrl.multiple_angle_read; 
        // 电源电压mV（小端序）
        int16_t voltage = roundf(1000.0f * VBUS_FACTOR * foc_ctrl.power_voltage_filter); 

        uint8_t rs485_tx[5+12];
        rs485_tx[0] = 0xff;
        rs485_tx[1] = 0xff;
        rs485_tx[2] = sizeof(rs485_tx);
        rs485_tx[3] = 0xff; // 固定为0xff，表示表示电机控制器发给主机的数据
        rs485_tx[4] = cmd_count;
        rs485_tx[5] = max_error & 0xff;
        rs485_tx[6] = (max_error >> 8) & 0xff;
        rs485_tx[7] = fifo_status & 0xff;
        rs485_tx[8] = (fifo_status >> 8) & 0xff;
        rs485_tx[9] = temperature;
        rs485_tx[10] = pos & 0xff;
        rs485_tx[11] = (pos >> 8) & 0xff;
        rs485_tx[12] = (pos >> 16) & 0xff;
        rs485_tx[13] = (pos >> 24) & 0xff;
        rs485_tx[14] = voltage & 0xff;
        rs485_tx[15] = (voltage >> 8) & 0xff;
        rs485_tx[16] = crc8(rs485_tx, sizeof(rs485_tx) - 1);

        // 等待足够长的时间，以便主机完成485方向切换
        sleep_until(delayed_by_us(uart_rx_time, RX_DELAY));
        gpio_put(UART_485_EN, 1);
        uart_write_blocking(UART_ID, rs485_tx, sizeof(rs485_tx));
        uart_tx_wait_blocking(UART_ID);
        gpio_put(UART_485_EN, 0);
        need_ack = false;
    }else if(cmd == RS485_CMD_ENABLE && pkg_length == 5){
        // 使能电机
        rs485_cmd_count ++;
        foc_ctrl.multiple_angle_target = foc_ctrl.multiple_angle_read;
        foc_ctrl.mode = MODE_POSITION_WITH_TRAPEZOID;
        need_ack = true;
    }else if(cmd == RS485_CMD_DISABLE && pkg_length == 5){
        // 禁用电机
        rs485_cmd_count ++;
        foc_ctrl.mode = MODE_CURRENT;
        sleep_us(500);// 等待至少1个控制周期，防止设置iq、id后又被修改
        foc_ctrl.iq_target = 0;
        foc_ctrl.id_target = 0;
        // trap_first_run = true;
        need_ack = true;
    }else if(cmd == RS485_CMD_RESET && pkg_length == 5){
        // 复位控制板
        rs485_cmd_count ++;
        watchdog_enable(10, 1);
        need_ack = true;
    }else if(cmd == RS485_CMD_CLEAR_FIFO && pkg_length == 5){
        fifo_clear(&foc_fifo);
        rs485_cmd_count ++;
        need_ack = true;
    }else if(cmd == RS485_CMD_SYNC && pkg_length == 1+5){
        // 判断是否要开启这个电机的同步控制
        // 如果想同步控制电机1和电机2，发送(1<<1)|(1<<2)即可
        // P[4] bit 7 : 堵转时是否暂停，==1暂停
        // P[4] bit 4 : 控制电机4
        // P[4] bit 3 : 控制电机3
        // P[4] bit 2 : 控制电机2
        // P[4] bit 1 : 控制电机1
        if( (p[4] & (1<<rs485_id)) ){
            // p[4]的最高位，表示堵转时是否要暂停运动控制
            foc_ctrl.stop_when_block         = p[4] & (1<<7) ? true : false;
            foc_ctrl.max_error = 0;
            foc_ctrl.block = false;
            foc_ctrl.trap_run = true;
            need_ack = true;
        }else{
            need_ack = false;
        }
        // printf("RS485_CMD_SYNC\n");
        rs485_cmd_count ++;
        
    }else if(cmd == RS485_CMD_FIFO && pkg_length >= 8+5){
        int succ = decompress_and_write_fifo(&p[4], pkg_length - 5, &foc_fifo);
        // printf("RS485_CMD_FIFO\n");
        if(succ == 0){
            rs485_cmd_count ++;
            need_ack = true;
        }else{
            printf("decompress_and_write_fifo fail with %d\n", succ);
        }
    }else{
        printf("CMD_UNKNOWN\n");
        return 4;
    }
    // 收到指令后电机回复ACK，如果是广播操作，不回复
    // 如果需要确认指令接收情况，可以检查cmd_count
    if(need_ack && id != 0){
        // 响应数据结构（固定5字节）：
        uint8_t rs485_tx[5];
        rs485_tx[0] = 0xff;
        rs485_tx[1] = 0xff;
        rs485_tx[2] = sizeof(rs485_tx);
        rs485_tx[3] = 0xff;
        rs485_tx[4] = crc8(rs485_tx, sizeof(rs485_tx) - 1);
        // 等待足够长的时间，以便主机完成485方向切换
        sleep_until(delayed_by_us(uart_rx_time, RX_DELAY));
        gpio_put(UART_485_EN, 1);
        uart_write_blocking(UART_ID, rs485_tx, sizeof(rs485_tx));
        uart_tx_wait_blocking(UART_ID);
        gpio_put(UART_485_EN, 0);
    }
    return 0;
}
int main(void)
{
    // ------------------------------- 初始化 -------------------------------
    stdio_init_all();
    memset((void*)&foc_ctrl, 0, sizeof(foc_ctrl_struct));
    memset((void*)calibration, 0, (TOTAL_SECTOR+1) * sizeof(int16_t));
    //resistor A = 1.968 ohm, resistor B = 2.017 ohm
    //inductance A = 4.037 mH, inductance B = 4.147 mH
    //design_bandwidth = 1800.00, Suggest kp = 185553760, ki = 3614336
    // 如果flash中存在保存的参数，以下默认设置会被覆盖
    foc_ctrl.curr_kp = 185553760;
    foc_ctrl.curr_ki = 3614336;
    foc_ctrl.speed_kp = 90000;
    foc_ctrl.speed_ki = 3000;
    foc_ctrl.pos_kp = 1300;
    foc_ctrl.pos_kd = 0; // 速度环单纯P控制就行，如果不稳定，可用适量增加微分环节
    foc_ctrl.enable_ff = true;
    foc_ctrl.psi = 0.004463f; 
    foc_ctrl.Ld = 6.0e-3f;
    foc_ctrl.Lq = 4.1e-3f;

    foc_ctrl.low_speed_id_current = LOW_SPEED_CURRENT;

    // 初始化位置信息FIFO
    fifo_init(&foc_fifo);

    // 485通信部分
    gpio_init(UART_485_EN);
    gpio_set_dir(UART_485_EN, GPIO_OUT);
    uart_init(UART_ID, 1000000); // 1Mbps
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_pulls(UART_RX_PIN, true, false);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    //uart_set_fifo_enabled(UART_ID, false); 默认FIFO是开启状态
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    // 加载FLASH中的配置
    shell_load_cmd(0, 0);

    // 将core1配置为高优先级，尝试解决USB通信时设置PWM耗时过长的问题
    // 完全没有效果，不管了，这个功能只在调试阶段用，不影响正常使用
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

    // 开启第二个核心
    multicore_launch_core1(core1_entry);
    while(!foc_ctrl.foc_controller_ready){
        sleep_ms(1);
    }

    // 交互调试Shell
    shell_init();
    while(true)
    {
        int c =  getchar_timeout_us(0);
        if(c != PICO_ERROR_TIMEOUT){
            shell(c);
        }
        if(uart_rx){
            uart_rx = false;
            decode_uart_rx_date(uart_rx_buffer);
        }
    }


    return 0;
}
