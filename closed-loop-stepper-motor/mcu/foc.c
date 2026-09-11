#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pin_def.h"
#include "foc.h"

#define IN_RAM(x) __not_in_flash_func(x)

int debug_data_buf[DEBUG_BUF_LEN][5];
volatile foc_ctrl_struct foc_ctrl;

static uint16_t adc_buf_a[CAPTURE_DEPTH];
static uint16_t adc_buf_b[CAPTURE_DEPTH];
// usr_adc_buf_a == false 正在写adc_buf_b，可以读取adc_buf_a
// usr_adc_buf_a == true 正在写adc_buf_a，可以读取adc_buf_b
static volatile bool usr_adc_buf_a = false;
static volatile bool dma_finish = false;

static uint dma_chan;
int16_t calibration[TOTAL_SECTOR+1]; 
fifo_t foc_fifo;

// -------------------- FIFO逻辑，用于缓存运动控制数据 --------------------
void fifo_init(fifo_t *f) 
{
    f->read_index = 0;
    f->write_index = 0;
    f->count = 0;

    spin_lock_t fifo_lock_instance = spin_lock_claim_unused(true);
    f->lock = spin_lock_init(fifo_lock_instance);
}

void fifo_clear(fifo_t *f) 
{
    uint32_t save = spin_lock_blocking(f->lock);
    f->read_index = 0;
    f->write_index = 0;
    f->count = 0;
    spin_unlock(f->lock, save);
}

uint32_t fifo_count(fifo_t *f) 
{
    uint32_t save = spin_lock_blocking(f->lock);
    uint32_t cnt = f->count;
    spin_unlock(f->lock, save);
    return cnt;
}

static bool IN_RAM(fifo_read)(fifo_t *f, int32_t *data, uint8_t *current) 
{
    uint32_t save = spin_lock_blocking(f->lock);
    
    if (f->count == 0) {
        spin_unlock(f->lock, save);
        return false;
    }

    *data = f->buffer[f->read_index];
    *current = f->current_buffer[f->read_index];
    f->read_index = (f->read_index + 1) & FIFO_MASK;
    f->count--;
    spin_unlock(f->lock, save);
    return true;
}

static uint32_t fifo_write(fifo_t *f, const int32_t *data, const uint8_t *current_data, uint32_t num) 
{
    uint32_t save = spin_lock_blocking(f->lock);
    
    uint32_t free_space = FIFO_DEPTH - f->count;
    if (free_space == 0) {
        spin_unlock(f->lock, save);
        return 0;
    }

    uint32_t to_write = (num <= free_space) ? num : free_space;
    uint32_t write_idx = f->write_index;
    uint32_t cont_space = FIFO_DEPTH - write_idx;

    if (to_write <= cont_space) {
        // 情况1：连续空间足够容纳全部写入数据
        memcpy(&f->buffer[write_idx], data, to_write * sizeof(int32_t));
        memcpy(&f->current_buffer[write_idx], current_data, to_write * sizeof(uint8_t));
        f->write_index = (write_idx + to_write) & FIFO_MASK;
    } else {
        // 情况2：数据需要分段写入（跨越缓冲区边界）
        memcpy(&f->buffer[write_idx], data, cont_space * sizeof(int32_t));
        memcpy(&f->current_buffer[write_idx], current_data, cont_space * sizeof(uint8_t));
        uint32_t remaining = to_write - cont_space;
        memcpy(f->buffer, data + cont_space, remaining * sizeof(int32_t));
        memcpy(f->current_buffer, current_data + cont_space, remaining * sizeof(uint8_t));
        f->write_index = remaining;
    }

    f->count += to_write;
    spin_unlock(f->lock, save);
    return to_write;
}

int decompress_and_write_fifo(const uint8_t* compressed, uint32_t compressed_size, fifo_t *fifo) 
{
    if (!compressed || compressed_size < 9) {
        return -1;
    }
    
    // 读取数据长度
    uint16_t data_length;
    memcpy(&data_length, compressed, 2);
    if(data_length == 0) {
        return -2;
    }
    
    // 读取第一个值
    int32_t value0;
    memcpy(&value0, compressed + 2, 4);
    
    // 读取第一次差分（即使数据长度为1，这个项目也存在，计算后舍弃即可）
    int16_t first_diff;
    memcpy(&first_diff, compressed + 6, 2);
    int32_t value1 = value0 + first_diff;
    
    // 位流解析初始化
    uint32_t byte_index = 8;
    uint32_t bit_buffer = 0;
    int bit_count = 0;
    int32_t prev_value = value1;       // 上一个输出的值
    int32_t prev_diff = first_diff;    // 上一个差分值
    
    // 使用局部数组缓存32个值和对应的电流值
    int32_t data_buffer[32];
    uint8_t current_buffer[32];
    uint32_t buffer_index = 0;
    uint8_t current_value = 0;
    
    // 处理第一个电流指令（在第一个差分之后）
    if (data_length > 1) {
        // 读取6位（3位标记 + 3位电流值）
        while (bit_count < 6) {
            if (byte_index >= compressed_size) {
                return -3;
            }
            bit_buffer = (bit_buffer << 8) | compressed[byte_index++];
            bit_count += 8;
        }
        
        // 提取标记和电流值
        uint8_t marker = (bit_buffer >> (bit_count - 3)) & 0x07;
        bit_count -= 3;
        current_value = (bit_buffer >> (bit_count - 3)) & 0x07;
        // printf("current=%d\n", current_value);
        bit_count -= 3;
        
        // 验证标记
        if (marker != 0x7) {
            return -4;
        }
    }
    
    // 将前两个值放入缓冲区（使用初始电流值）
    data_buffer[buffer_index] = value0;
    current_buffer[buffer_index++] = current_value;
    
    if(data_length >= 1){
        data_buffer[buffer_index] = value1;
        current_buffer[buffer_index++] = current_value;
    }
    
    // 解压剩余数据（从第三个数据开始）
    for (uint32_t i = 2; i < data_length; i++) {
        // 从位流中提取3位
        while (bit_count < 3) {
            if (byte_index >= compressed_size) {
                return -5;
            }
            bit_buffer = (bit_buffer << 8) | compressed[byte_index++];
            bit_count += 8;
        }
        
        // 获取高位3位
        uint8_t bits = (bit_buffer >> (bit_count - 3)) & 0x07;
        bit_count -= 3;
        
        // 检查是否为电流调整指令
        if (bits == 0x7) {
            // 电流指令 - 再提取3位电流值
            while (bit_count < 3) {
                if (byte_index >= compressed_size) {
                    return -6;
                }
                bit_buffer = (bit_buffer << 8) | compressed[byte_index++];
                bit_count += 8;
            }
            
            current_value = (bit_buffer >> (bit_count - 3)) & 0x07;
            // printf("current=%d\n", current_value);
            bit_count -= 3;
            
            // 继续提取下一个3位（实际加速度值）
            while (bit_count < 3) {
                if (byte_index >= compressed_size) {
                    return -7;
                }
                bit_buffer = (bit_buffer << 8) | compressed[byte_index++];
                bit_count += 8;
            }
            
            bits = (bit_buffer >> (bit_count - 3)) & 0x07;
            bit_count -= 3;
        }
        
        // 验证加速度值范围
        if (bits > 6) {
            return -8;
        }
        
        // 计算当前差分和数值
        int32_t accel = (int32_t)bits - 3;
        int32_t curr_diff = prev_diff + accel;
        int32_t curr_value = prev_value + curr_diff;
        
        // 将当前值和电流值放入缓冲区
        data_buffer[buffer_index] = curr_value;
        current_buffer[buffer_index++] = current_value;
        
        // 更新状态
        prev_diff = curr_diff;
        prev_value = curr_value;
        
        // 当缓冲区满32个值时，写入FIFO
        if (buffer_index == 32) {
            uint32_t written = fifo_write(fifo, data_buffer, current_buffer, 32);
            if (written != 32) {
                return -9;
            }
            buffer_index = 0;
        }
    }
    
    // 写入缓冲区中剩余的数据
    if (buffer_index > 0) {
        uint32_t written = fifo_write(fifo, data_buffer, current_buffer, buffer_index);
        if (written != buffer_index) {
            return -10;
        }
    }
    
    return 0;
}


// 通过不同的LED闪烁模式指示错误类型
static void error_code(int x)
{
    while (1)
    {
        for (int i = 3; i >= 0; i--)
        {
            gpio_put(LED1, 1);
            if (x & (1 << i))
            {
                sleep_ms(800);
            }
            else
            {
                sleep_ms(100);
            }
            gpio_put(LED1, 0);
            sleep_ms(500);
        }
        sleep_ms(3000);
    }
}

// 基于查表法的快速三角函数，表格大小为1/4个sin周期。
// 输入范围0-1023，代表0-2pi角度。
// 输出范围-16384 ～ 16384，代表-1～1。
static int sin_cos_table[256];// 三角函数表
static void init_sin_table(void) {
    for (int i = 0; i < 256; i++) {
        // 使用 float 计算角度和正弦值
        float angle = (float)i * (float)M_PI / 2.0f / 255.0f;
        float sin_val = sinf(angle); // 单精度浮点计算
        sin_cos_table[i] = (int)(sin_val * 16384.0f + 0.5f); // float 四舍五入
    }
}
static int IN_RAM(fast_sin)(int x) {
    int quadrant = (x >> 8) & 3;
    int index = x & 0xFF;
    int value;

    switch (quadrant) {
        case 0:
            value = sin_cos_table[index];
            break;
        case 1:
            value = sin_cos_table[255 - index];
            break;
        case 2:
            value = -sin_cos_table[index];
            break;
        case 3:
            value = -sin_cos_table[255 - index];
            break;
        default:
            value = 0;
    }

    return value;
}
static int IN_RAM(fast_cos)(int x) {
    int adjusted_x = (x + 256) & 0x3FF; // 等价于 (x + 256) % 1024
    return fast_sin(adjusted_x);
}
// 编写5个数据求中位数的函数，需要高效率
// 读取上一周期的ADC数据后调用该函数，每个周期每个通道采集5个数据，求中位数
static int IN_RAM(get_adc_average)(uint16_t *buf, int offset) {
    uint16_t a = buf[offset];
    uint16_t b = buf[offset + 4];
    uint16_t c = buf[offset + 8];
    uint16_t d = buf[offset + 12];
    uint16_t e = buf[offset + 16];

    // 使用手动展开的插入排序对五个元素进行排序
    if (b < a) { uint16_t t = a; a = b; b = t; }

    if (c < b) {
        uint16_t t = b; b = c; c = t;
        if (b < a) { uint16_t t = a; a = b; b = t; }
    }

    if (d < c) {
        uint16_t t = c; c = d; d = t;
        if (c < b) {
            uint16_t t = b; b = c; c = t;
            if (b < a) { uint16_t t = a; a = b; b = t; }
        }
    }

    if (e < d) {
        uint16_t t = d; d = e; e = t;
        if (d < c) {
            uint16_t t = c; c = d; d = t;
            if (c < b) {
                uint16_t t = b; b = c; c = t;
                if (b < a) { uint16_t t = a; a = b; b = t; }
            }
        }
    }

    // 中位数为排序后的第三个元素
    return (int)c;
}
// 读取MT6816
// 2025-05-03开发故障检测功能
static bool IN_RAM(read_mt6816)(uint16_t *out) 
{
    const uint8_t reg[3] = {0x83, 0, 0};
    uint8_t buf[3];
    gpio_put(MT6816_CS, 0);// >100ns to SCK
    spi_write_read_blocking(SPI_PORT, reg, buf, 3);
    gpio_put(MT6816_CS, 1);// >32ns from SCK
    if(buf[2] & 2){
        // 磁场过弱，注意把径向磁铁，换成轴向磁铁，是不会触发磁场过弱故障的
        return false;
    }else if(__builtin_parity((buf[1] ^ buf[2]) & 0xFF)){
        // 偶校验失败
        return false;
    }else{
        *out = (uint16_t)(buf[1]<<6) | (buf[2]>>2);
        return true;
    }
}

// 每40us调用一次，1000RPM时，每次调用current_value的增量在11左右
// ENCODER_MAX为16384
// current_value为编码器的位置，取值范围0-16383
// angle为多圈编码器值，16384表示一圈
// speed是最近0.2*8ms angle的增量，如果要换算为RPM，speed in RPM = speed * (60000ms / 1.6ms)) / 16384
void IN_RAM(get_multiturn_angle_speed)(int current_value, volatile int *angle, int *speed) 
{
    //static int total_angle = 0;
    static int previous_value = 0;
    static int initialized = 0;
    // 速度计算相关静态变量
    static int delta_buffer[SPEED_BUFFER_SIZE] = {0};
    static int buffer_index = 0;
    static int buffer_count = 0;
    static int delta_sum = 0;
    if (!initialized) {
        previous_value = current_value;
        *angle = current_value;
        initialized = 1;
        *speed = 0;
        return;
    }

    // 计算原始增量并处理边界条件
    int delta_raw = current_value - previous_value;
    int delta = delta_raw;
    if (delta_raw > ENCODER_MAX/2) {
        delta = delta_raw - ENCODER_MAX;
    } else if (delta_raw < -ENCODER_MAX/2) {
        delta = delta_raw + ENCODER_MAX;
    } 

    // 更新滑动窗口缓冲区
    delta_sum += delta;
    if (buffer_count >= SPEED_BUFFER_SIZE) {
        delta_sum -= delta_buffer[buffer_index];
    }

    delta_buffer[buffer_index] = delta;
    buffer_index = (buffer_index + 1) % SPEED_BUFFER_SIZE;
    if (buffer_count < SPEED_BUFFER_SIZE) {
        buffer_count++;
    }
    

    // 更新总角度
    *angle += delta;
    previous_value = current_value;

    *speed = delta_sum;
}

static void IN_RAM(calibration_angle)(int input, int *electro, int *machanic)
{
    int sector = 0;
    int angle;
    int16_t a, b;
    // 处理跳变区
    if (input < calibration[0]) {
        input += ENCODER_MAX;
    }
    // 二分法查找扇区
    int left = 0;
    int right = TOTAL_SECTOR; // 数组大小是 TOTAL_SECTOR + 1
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if (calibration[mid] <= input) {
            sector = mid;
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    // 确定a和b的值，不需要处理最后一个扇区的情况
    // 已经提前处理了 calibration[200] = calibration[0] + ENCODER_MAX; 
    a = calibration[sector];
    b = calibration[sector + 1];
    
    // 计算角度
    angle = 256 * (input - a) / (b - a);
    angle += 256 * (sector % 4);
    // 根据返回类型调整结果, 确保在0-1023范围内
    *electro = angle % 1024;
    // angle % 1024 + 1024 * (sector / 4) 范围0-51199
    // (angle % 1024 + 1024 * (sector / 4)) * 8 / 25 范围0-16383
    *machanic = (angle % 1024 + 1024 * (sector / 4)) * 8 / 25; 
}
// 速度环PI控制器
static int IN_RAM(pi_ctrl)(int target, int input, int clamp_min, int clamp_max, int kp, int ki, int *integral)
{
    const int scale = 8192;
    int error = target - input;
    int p_term = (kp * error) / scale;
    int i_term = (*integral * ki) / scale;
    int output_unclamped = p_term + i_term;
    int output;
    // 限幅处理
    if (output_unclamped > clamp_max) {
        output = clamp_max;
    } else if (output_unclamped < clamp_min) {
        output = clamp_min;
    } else {
        output = output_unclamped;
    }
    // 积分抗饱和处理
    if (output == output_unclamped) {
        // 未饱和时正常积分，但是要限制幅度，防止integral * ki溢出
        if(abs(*integral) < 262144) *integral += error;
    } else {
        // 饱和时反算积分项防止windup
        if (ki != 0) { // 避免除零保护
            *integral = (output * scale - kp * error) / ki;
        }
    }
    return output;
}

// 位置环PD控制器
static int IN_RAM(pd_ctrl)(int target, int current, int clamp, int kp, int kd, int* prev_error, int ff) 
{
    const int scale = 8192;  // 与原PI控制器保持一致
    int error = target - current;
    
    // 计算微分项（误差变化率）
    int error_diff = error - *prev_error;
    *prev_error = error;
    
    // PD计算
    int p_term = (kp * error) / scale;
    int d_term = (kd * error_diff) / scale;
    int output = p_term + d_term + ff;
    
    // 限幅处理
    if (output > clamp) {
        output = clamp;
    } else if (output < -clamp) {
        output = -clamp;
    }
    return output;
}

// ---------------------------- DMA中断，每秒25K次，耗时2us左右 ----------------------------
static void IN_RAM(on_dma_finish)(void) 
{
    //gpio_put(LED1, 1);
    dma_hw->ints0 = 1u << dma_chan; // Clear the interrupt request.
    usr_adc_buf_a = !usr_adc_buf_a;
    uint16_t *capture_buf = usr_adc_buf_a ? adc_buf_a : adc_buf_b;
    dma_channel_set_write_addr(dma_chan, capture_buf, false);
    dma_channel_set_trans_count(dma_chan, CAPTURE_DEPTH, true);
    dma_finish = true;
    // 采样顺序4 0 1 2... (温度 B相 A相 电源)
    uint16_t *read_adc_buf = usr_adc_buf_a ? adc_buf_b : adc_buf_a;
    foc_ctrl.temperature_raw = read_adc_buf[0]; // 0 4 8 12 16
    foc_ctrl.i_beta  = get_adc_average(read_adc_buf, 1) - foc_ctrl.i_offset_beta;
    if(foc_ctrl.reverse_v_alpha){
        foc_ctrl.i_alpha = foc_ctrl.i_offset_alpha - get_adc_average(read_adc_buf, 2);
    }else{
        foc_ctrl.i_alpha = get_adc_average(read_adc_buf, 2) - foc_ctrl.i_offset_alpha;
    }
    int voltage = get_adc_average(read_adc_buf, 3);
    foc_ctrl.power_voltage_filter = (foc_ctrl.power_voltage_filter*7 + voltage)/8;
    foc_ctrl.power_voltage_raw = voltage;
    //gpio_put(LED1, 0);
}

// 支持32位输入的快速整数平方根
static uint32_t IN_RAM(isqrt_32bit)(uint32_t num) 
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30; // 从适合32位数的最高位开始
    // 调整bit到最大的可能幂
    while (bit > num) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (num >= res + bit) {
            num -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

// ---------------------------- PWM中断，每秒25K次，无USB通信时最长耗时12.5us左右 ----------------------------
void IN_RAM(on_pwm_wrap)(void) 
{
    bool need_debug = foc_ctrl.fill_debug_buf > 0 ? true : false; 
    int slice_num = pwm_gpio_to_slice_num(PWM_A1); // PWM0
    static int intg_d = 0, intg_q = 0;   // 电流环积分
    static int debug_buf_count = 0;
    static int count_by_5 = 0;
    if (pwm_get_irq_status_mask() & (1 << slice_num)) {
        // 清除中断标志
        pwm_clear_irq(slice_num);
        // 速度环更新标志
        if(++count_by_5 == 5){
            count_by_5 = 0;
            foc_ctrl.update_speed = true;
        }else{
            if(foc_ctrl.debug_div_5){
                need_debug = false;
            }
        }
        int angle=0, machanic_angle;
        uint16_t mt6816_angle;
        if(read_mt6816(&mt6816_angle)){
            calibration_angle(mt6816_angle, &angle, &machanic_angle);// 校准角度，耗时2.4us
            foc_ctrl.machanic_angle = machanic_angle;
            foc_ctrl.mt6816_angle_read = mt6816_angle;
            foc_ctrl.mt6816_fail_count = 0;
        }else{
            foc_ctrl.mt6816_fail_count ++;
        }
        
        foc_mode mode =  foc_ctrl.mode;
        int vd_out, vq_out;
        if(mode == MODE_VOLATGE){
            // -------------------------------- 开环电压控制 --------------------------------
            angle = foc_ctrl.electro_angle;
            vd_out = foc_ctrl.vd;
            vq_out = foc_ctrl.vq;
        }else{
            foc_ctrl.electro_angle = angle;
            // -------------------------------- 电流环 --------------------------------
            // 根据实测电流计算id、iq，i_alpha，i_beta数据来自on_dma_finish
            int a = foc_ctrl.i_alpha;
            int b = foc_ctrl.i_beta;
            int id_meas =  a * fast_cos(angle) / 16384 + b * fast_sin(angle) / 16384; // Id ​= Iα​cos(θ)  + Iβ​sin(θ) 
            int iq_meas = -a * fast_sin(angle) / 16384 + b * fast_cos(angle) / 16384; // Iq ​= −Iα​sin(θ) + Iβ​cos(θ)
            foc_ctrl.id = id_meas;
            foc_ctrl.iq = iq_meas;            
            // DQ轴联合PI控制器，带矢量限幅
            // id_target   d轴目标电流,±2048
            // iq_target   q轴目标电流,±2048
            // id_meas     d轴实测电流,±2048
            // iq_meas     q轴实测电流,±2048
            // kp          比例系数，例如90000
            // ki          积分系数，例如1712，需要小于8192
            // pwm_max     PWM最大幅值（2400）
            // intg_d      d轴积分器状态
            // intg_q      q轴积分器状态
            // vd_out      输出d轴电压，±2400
            // vq_out      输出q轴电压，±2400
            int volt = foc_ctrl.power_voltage_filter;
            int kp = foc_ctrl.curr_kp / volt; // 根据电源电压动态调整kp
            int ki = foc_ctrl.curr_ki / volt;
            int id_target = foc_ctrl.id_target;
            int iq_target = foc_ctrl.iq_target;
            int w_Lq = foc_ctrl.w_Ld;
            int w_Ld = foc_ctrl.w_Lq;
            int w_psi = foc_ctrl.w_psi;
            const int scale = 8192; // PI控制器缩放系数
            const int pwm_max = PWM_MAX;
            // Step 1: 计算未限幅的PI输出
            int error_d = id_target - id_meas;
            int error_q = iq_target - iq_meas;
            
            // d轴PI计算
            int p_term_d = (kp * error_d) / scale;
            if(abs(intg_d) < 262144) intg_d += error_d;// 限制幅度，实测在40000左右
            int i_term_d = (intg_d * ki) / scale;
            int vd_ff = -w_Lq * iq_target / scale; // 前馈，要用目标电流，如果用实测电流会导致环路不稳定
            int vd_unclamp = p_term_d + i_term_d + vd_ff;
            if(vd_unclamp > 32767) vd_unclamp = 32767; // sqrt((2^31 - 1) / 2) = 32767.999992370605
            // q轴PI计算
            int p_term_q = (kp * error_q) / scale;
            if(abs(intg_q) < 262144) intg_q += error_q;
            int i_term_q = (intg_q * ki) / scale;
            int vq_ff = (w_psi + w_Ld * id_target) / scale;// 前馈
            int vq_unclamp = p_term_q + i_term_q + vq_ff;
            if(vq_unclamp > 32767) vq_unclamp = 32767;

            // Step 2: 矢量幅度计算，sq_mag最大2147352578 < 2^31-1
            int sq_mag = vd_unclamp*vd_unclamp + vq_unclamp*vq_unclamp;
            int max_sq = pwm_max * pwm_max; // 2400*2400 = 5760000 < 2^31-1

            // Step 3: 幅度限制判断
            if(sq_mag > max_sq) 
            {
                // 快速整数平方根
                uint32_t sqrt_val = isqrt_32bit((uint32_t)sq_mag);
                int scale_factor = (pwm_max * scale) / sqrt_val;
                
                // 矢量缩放
                vd_out = (vd_unclamp * scale_factor) / scale;
                vq_out = (vq_unclamp * scale_factor) / scale;
                
                // Step 4: 联合抗饱和处理
                // 推导公式：intg = (v_actual*scale - kp*error)/ki

                // 2025-5-1：抗饱和时考虑前馈项vd_ff， vq_ff
                // 解决以下的BUG：
                // 不加前馈没有问题，加上vq_ff后容易触发幅度限制，触发幅度限制后vq_ff正常，但是i_term_q会异常增大，导致幅度限制会一直被触发
                if(ki != 0) {
                    intg_d = (vd_out * scale - kp * error_d - vd_ff * scale) / ki;
                    intg_q = (vq_out * scale - kp * error_q - vq_ff * scale) / ki;
                }
                //*sqrt_vd_vq = pwm_max;
            } else {
                vd_out = vd_unclamp;
                vq_out = vq_unclamp;
                //*sqrt_vd_vq = isqrt_32bit((uint32_t)sq_mag);
            }
            foc_ctrl.vq = vq_out;
            foc_ctrl.vd = vd_out;
            foc_ctrl.vq_ff = vq_ff;
            foc_ctrl.vd_ff = vd_ff;
        }
        // 计算并且更新PWM占空比，耗时3us，同时使用USB会大幅度延迟这段代码的运行时间
        // 计算PWM占空比（驱动电压），
        int v_alpha = vd_out * fast_cos(angle) / 16384 - vq_out * fast_sin(angle) / 16384; // Vα ​= Vd​cos(θ)  − Vq​sin(θ)  
        int v_beta =  vd_out * fast_sin(angle) / 16384 + vq_out * fast_cos(angle) / 16384; // Vβ ​= Vd​sin(θ)  + Vq​cos(θ)
        foc_ctrl.v_alpha = v_alpha;
        foc_ctrl.v_beta = v_beta;
        
        // EG3013S驱动器PWM控制逻辑
        // 0    0    下管慢速衰减模式（Slow Decay）
        // 0    1    反向 (OUT2 → OUT1)
        // 1    0    正向 (OUT1 → OUT2)
        // 1    1    上管慢速衰减模式（不使用）
        // 不允许长时间将IN1或者IN2配置为1，会影响自举升压电路的工作
        if(foc_ctrl.reverse_v_alpha){
            // 调换A1 A2的顺序
            if(v_alpha >= 0){
                pwm_set_gpio_level(PWM_A2, v_alpha);
                pwm_set_gpio_level(PWM_A1, 0);
            }else{
                pwm_set_gpio_level(PWM_A2, 0);
                pwm_set_gpio_level(PWM_A1, -v_alpha);
            }
        }else{
            if(v_alpha >= 0){
                pwm_set_gpio_level(PWM_A1, v_alpha);
                pwm_set_gpio_level(PWM_A2, 0);
            }else{
                pwm_set_gpio_level(PWM_A1, 0);
                pwm_set_gpio_level(PWM_A2, -v_alpha);
            }
        }

        if(v_beta >= 0){
            pwm_set_gpio_level(PWM_B1, v_beta);
            pwm_set_gpio_level(PWM_B2, 0);
        }else{
            pwm_set_gpio_level(PWM_B1, 0);
            pwm_set_gpio_level(PWM_B2, -v_beta);
        }
        // 调试逻辑
        if(need_debug){
            int *p = debug_data_buf[debug_buf_count];
            for(int i=0;i<5;i++){
                volatile int *address = foc_ctrl.debug_date_address[i];
                p[i] = *address;
            }
            if(++debug_buf_count == DEBUG_BUF_LEN || debug_buf_count == foc_ctrl.fill_debug_buf){
                debug_buf_count = 0;
                foc_ctrl.fill_debug_buf = 0;
            }
        }
    }
}

static inline float IN_RAM(pow2f)(float x) {
    return x * x;
}

// 速度环调试顺序：
// 先关积分（Ki=0）：逐渐增大 Kp 直至系统出现轻微震荡，取该值的60%~70%作为最终Kp。
// 后加积分：从 Kp/50 开始增加 Ki，直到稳态误差消除且无明显超调。
// 预期现象与调整：
// 响应过慢 → 增大 Kp（每次+20%）
// 震荡/超调 → 减小 Kp 或增大 Ki
// 稳态误差大 → 增大 Ki（每次+50%）

// 电流环​​：25kHz（on_pwm_wrap，全整数运算优化）
// ​​速度环​​：5kHz
// ​​位置环​​：5kHz
static void IN_RAM(foc_loop)(void) 
{            
    // 速度环积分
    int int_error_speed = 0; 
    // 位置环差分误差
    int prev_angle_error = 0;
    // 位置控制
    int target_pos_last = 0;
    bool target_pos_last_valid = false;
    // 电流控制
    int max_current = MAX_CURRENT;
    while(true){
        while (!foc_ctrl.update_speed);
        foc_ctrl.update_speed = false;
        // 过压保护检测
        if (foc_ctrl.power_voltage_raw >= PROTECT_OVER_VOLTAGE) {
            foc_ctrl.mode = MODE_OVER_VOLTAGE_PROTECT;
            break;
        }
        // 欠压保护检测
        else if (foc_ctrl.power_voltage_raw <= PROTECT_UNDER_VOLTAGE) {
            foc_ctrl.mode = MODE_UNDER_VOLTAGE_PROTECT;
            break;
        }
        // 过流保护检测
        if (abs(foc_ctrl.i_alpha) >= PROTECT_OVER_CURRENT || abs(foc_ctrl.i_beta) > PROTECT_OVER_CURRENT) {
            foc_ctrl.mode = MODE_OVER_CURRENT_PROTECT;
            break;
        }
        // 过速保护检测
        if (foc_ctrl.speed_read >= PROTECT_OVER_SPEED) {
            foc_ctrl.mode = MODE_OVER_SPEED_PROTECT;
            break;
        }
        // 过热保护检测
        if (foc_ctrl.temperature_raw <= PROTECT_OVER_TEMPERATURE) {
            foc_ctrl.mode = MODE_OVER_TEMPERATURE_PROTECT;
            break;
        }
        if(foc_ctrl.mt6816_fail_count >= PROTECT_MT6816_ERROR_COUNT){
            foc_ctrl.mode = MODE_MT6816_FAIL_PROTECT;
            break;
        }
        int speed;
        // 获取位置和速度，multiple_angle每一圈是16384，支持多圈累计
        // speed是最近0.8ms multiple_angle的增量
        // 如果要换算为RPM： speed * (60000ms / 0.8ms)) / 16384 = speed * 9375/2048
        // 如果要换算为电角速度rad/s：speed * 9375/2048 * (2 * pi * 50) / 60 , 50极对数，60秒
        
        get_multiturn_angle_speed(foc_ctrl.machanic_angle, &foc_ctrl.multiple_angle_read, &speed);
        foc_ctrl.speed_read = speed;
        foc_mode mode = foc_ctrl.mode;
        int target_velocity_feed_forward = 0;// 位置环的速度前馈量
        // -------------------------------- 位置规划器,从FIFO中读取位置曲线并且移动到对应的位置 --------------------------------
        if(mode == MODE_POSITION_WITH_TRAPEZOID && foc_ctrl.trap_run){
            // 获取目标位置
            int32_t target_pos = 0;
            uint8_t target_current = 0;// 0 - 7
            bool succ = fifo_read(&foc_fifo, &target_pos, &target_current);
            if(succ){
                // 获取目标速度
                int target_speed;
                if(target_pos_last_valid){
                    target_speed = target_pos - target_pos_last;
                    target_pos_last = target_pos;
                }else{
                    target_speed = 0;
                    target_pos_last = target_pos;
                    target_pos_last_valid = true;
                }
                // 设置目标位置和速度前馈量
                foc_ctrl.multiple_angle_target = target_pos;
                target_velocity_feed_forward = target_speed * SPEED_BUFFER_SIZE;
                // 获取当前的位置误差
                int max_pos_error = abs(target_pos - foc_ctrl.multiple_angle_read);
                if(max_pos_error > foc_ctrl.max_error){
                    foc_ctrl.max_error = max_pos_error;
                }
                // 获取电流配置
                max_current = MAX_CURRENT * (target_current + 1) / 8;
                // 调试代码，指示当前的电流大小
                gpio_put(LED2, target_current > 3 ? 0 : 1);
                // 如果发生堵转
                if(max_pos_error >= BLOCK_DETECT_VALUE){
                    // 停止并且保持堵转前的位置，用于实现无限位开关归零功能
                    if(foc_ctrl.stop_when_block){
                        foc_ctrl.multiple_angle_target = foc_ctrl.multiple_angle_read;
                        foc_ctrl.trap_run = false;
                        target_pos_last_valid = false;
                    }
                    // 设置堵转标志
                    foc_ctrl.block = true;
                }
            }else{
                foc_ctrl.trap_run = false;
                target_pos_last_valid = false;
            }
        }
        // -------------------------------- 位置环 --------------------------------
        // 输入 foc_ctrl.multiple_angle_target
        // 输出 foc_ctrl.speed_target
        if(mode == MODE_POSITION || mode == MODE_POSITION_WITH_TRAPEZOID){
            foc_ctrl.speed_target = pd_ctrl(foc_ctrl.multiple_angle_target, foc_ctrl.multiple_angle_read,
                                            MAX_SPEED, foc_ctrl.pos_kp, foc_ctrl.pos_kd,
                                            &prev_angle_error,target_velocity_feed_forward);
        }
        // -------------------------------- 速度环 --------------------------------
        // 输入 foc_ctrl.speed_target
        // 输出 foc_ctrl.id_target，foc_ctrl.iq_target
        if(mode == MODE_POSITION || mode == MODE_POSITION_WITH_TRAPEZOID || mode == MODE_SPEED){
            // -------------------------------- 速度环PI控制 --------------------------------
            int is_target = pi_ctrl(foc_ctrl.speed_target, speed,
                                    -max_current, max_current, 
                                    foc_ctrl.speed_kp, foc_ctrl.speed_ki, 
                                    &int_error_speed);
            if(abs(is_target) > LED_ON_CURRENT){
                gpio_put(LED1, 1);
            }else{
                gpio_put(LED1, 0);
            }
            // -------------------------------- 弱磁，根据转速分配Iq、Id --------------------------------
            const float current_factor = 3.3f / 2048;// 到电流（A）的换算系数
            const float psi = foc_ctrl.psi; // 磁链
            const float eta = 0.90 * (float)PWM_MAX / (float)(PWM_TOP+1);// 留余量，防止电机参数不准确时出现问题
            const float Ld = foc_ctrl.Ld;// 电感 Ld和Lq
            const float Lq = foc_ctrl.Lq;
            const float max_weakening = 0.8f; // 最大抵消永磁体磁场的80%，全部抵消会变得几乎没有转矩
            const float id_min = - psi * max_weakening / Ld;
            // (Ld *​ id​ + ψ)² + (Lq *​ iq​)² = (Vmax​ / ω​)²
            float Vmax = foc_ctrl.power_voltage_filter * VBUS_FACTOR * eta;
            float w = speed * SPEED_TO_W;
            float w_abs = fabsf(w);
            float iq_target = is_target * current_factor;
            float iq = 0, id = 0;
            float low_speed_current = foc_ctrl.low_speed_id_current;
            if (w_abs > 1e-6) { // 避免除零
                float ld_id_psi = pow2f(Vmax / w_abs) - pow2f(Lq * iq_target);
                if(pow2f((1 - max_weakening) * psi) <= ld_id_psi){
                    // 恒力矩
                    iq = iq_target;
                    // ld_id_psi = (Ld *​ id​ + ψ)², 求id​
                    id = ( sqrtf(ld_id_psi) - psi ) / Ld;
                    // 限制id不大于LOW_SPEED_CURRENT
                    if (id > low_speed_current) id = low_speed_current;
                }else{
                    // 限制力矩避免电压饱和
                    id = id_min;
                    float Lqiq = pow2f(Vmax / w_abs) - pow2f(Ld * id_min + psi);
                    if(Lqiq >= 0){
                        iq = sqrtf(Lqiq) / Lq;
                        if(iq_target < 0) iq = -iq;
                    }else{
                        iq = 0; // 过速保护
                    }
                }
            }else{
                // 零速控制策略
                iq = iq_target;
                id = low_speed_current;
            }

            // foc_ctrl.is_target = is_target;
            foc_ctrl.id_target = roundf(id / current_factor);
            foc_ctrl.iq_target = roundf(iq / current_factor);
            //调试电源电压利用率
            //foc_ctrl.debug = isqrt_32bit(foc_ctrl.vd * foc_ctrl.vd + foc_ctrl.vq * foc_ctrl.vq) * 100 / PWM_MAX;
            //foc_ctrl.debug = int_error_speed;

            // -------------------------------- 电流环前馈，抵消反电动势 --------------------------------
            // 根据当前转速，更新电流环前馈变量，只有开启速度环才有这个功能
            // id由ADC原始值换算为A，需要乘以系数current_factor
            // 计算出的电压由V换算为PWM占空比，需要乘以系数 (PWM_TOP+1)/(VBUS_FACTOR * foc_ctrl.power_voltage_filter)
            // 缩放系数8192
            // 例如VBUS_FACTOR * foc_ctrl.power_voltage_filter = 24V
            // PWM_TOP + 1 = 2500
            // current_factor = 0.001611
            if(foc_ctrl.enable_ff){
                float factor_v = 8192.0f * (PWM_TOP + 1) / (VBUS_FACTOR * (float)foc_ctrl.power_voltage_filter);
                float factor_i_v = current_factor * factor_v;
                // q轴：ω*Ld*id + ω*ψ
                foc_ctrl.w_Ld = w * Ld * factor_i_v;
                foc_ctrl.w_psi = w * psi * factor_v;
                // d轴：ω*Lq*iq
                foc_ctrl.w_Lq = w * Lq * factor_i_v;
            }else{
                foc_ctrl.w_Ld = 0;
                foc_ctrl.w_Lq = 0;
                foc_ctrl.w_psi = 0;
            }
        }else{
            foc_ctrl.w_Ld = 0;
            foc_ctrl.w_Lq = 0;
            foc_ctrl.w_psi = 0;
        }
    }
}

static float wait_and_get_power_voltage(void)
{
    adc_select_input(2);
    const float conversion_factor = VBUS_FACTOR;
    const float threshold = 0.02f; // 2%偏差阈值
    const uint32_t timeout_ms = 1000; // 1秒超时
    const uint32_t sample_interval_ms = 50; // 采样间隔50ms
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float last_voltage = 0.0f;
    bool first_reading = true;
    while (true) {
        // 读取ADC值
        int result = 0;
        for (int i = 0; i < 1024; i++) {
            result += adc_read();
        }
        result /= 1024;
        float current_voltage = result * conversion_factor;
        if (first_reading) {
            first_reading = false;
            last_voltage = current_voltage;
            sleep_ms(sample_interval_ms);
            continue;
        }
        // 检查电压是否稳定
        float deviation = fabsf(current_voltage - last_voltage) / last_voltage;
        if (deviation < threshold) {
            return current_voltage;
        }
        last_voltage = current_voltage;
        // 检查超时
        if (to_ms_since_boot(get_absolute_time()) - start_time >= timeout_ms) {
            return current_voltage; // 超时返回最后一次读数
        }
        sleep_ms(sample_interval_ms);
    }
}
void core1_entry(void) 
{
    int i_offset_alpha = 0; // 0 - 4095 A相位电流零点
    int i_offset_beta = 0;  // 0 - 4095 B相位电流零点
    init_sin_table();
    gpio_init(LED1);
    gpio_set_dir(LED1, GPIO_OUT);
    gpio_init(LED2);
    gpio_set_dir(LED2, GPIO_OUT);
    // ------------------------------- 配置ADC ------------------------------- 
    adc_gpio_init(GPIO_IOUT_A);
    adc_gpio_init(GPIO_IOUT_B);
    adc_gpio_init(GPIO_VBUS_SNS);
    adc_init();
    adc_set_temp_sensor_enabled(true);
    // 等待电源电压稳定
    wait_and_get_power_voltage();
    adc_select_input(2);
    foc_ctrl.power_voltage_raw = adc_read();
    // 获取电流采样的零点
    adc_select_input(1);
    for (int i = 0; i < 1024; i++) {
        i_offset_alpha += adc_read();
    }
    foc_ctrl.i_offset_alpha = i_offset_alpha / 1024;
    adc_select_input(0);
    for (int i = 0; i < 1024; i++) {
        i_offset_beta += adc_read();
    }
    foc_ctrl.i_offset_beta = i_offset_beta / 1024;
    // 配置采样顺序4 0 1 2... (温度 B相 A相 电源)
    // 参考datasheet 4.9.2.3. Sampling Multiple Inputs
    adc_select_input(4);
    adc_set_round_robin(0b10111); // 0 1 2 4
    adc_set_clkdiv(0);// 500K采样率
    adc_fifo_setup(true, true, 1, false, false);
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);
    dma_channel_configure(dma_chan, &cfg,
        NULL,           // dst，每次传输之前填写
        &adc_hw->fifo,  // src
        0,              // transfer count，每次传输之前填写
        false           // not start immediately
    );
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, on_dma_finish);
    // 提升优先级，解决通过USB传输大量数据时，ADC采集的数据会偶尔出错，导致欠压保护的问题
    irq_set_priority(DMA_IRQ_0, 0);
    irq_set_enabled(DMA_IRQ_0, true);
    on_dma_finish(); // 手动调用一次，初始化DMA的目的寄存器和传输数量寄存器
    dma_finish = false;
    // ------------------------------- 配置PWM ------------------------------- 
    gpio_set_function(PWM_A1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_A2, GPIO_FUNC_PWM);
    gpio_set_function(PWM_B1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_B2, GPIO_FUNC_PWM);

    uint slice_num[2];
    slice_num[0] = pwm_gpio_to_slice_num(PWM_A1); // PWM0
    slice_num[1] = pwm_gpio_to_slice_num(PWM_B1); // PWM4

    for(int i = 0; i < 2; i++){
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&config, 1);
        pwm_config_set_phase_correct(&config, true);
        pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
        pwm_config_set_output_polarity(&config, false, false);
        pwm_config_set_wrap(&config, PWM_TOP);
        pwm_init(slice_num[i], &config, false);
        if(i == 0){
            pwm_clear_irq(slice_num[i]);
            pwm_set_irq_enabled(slice_num[i], true);
            irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);// 绑定中断处理函数
            irq_set_enabled(PWM_IRQ_WRAP, true);
        } 
    }
    // ------------------------------- 配置SPI ------------------------------- 
    spi_init(SPI_PORT, 15625000); // 15.625MHz.
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(MT6816_MISO, GPIO_FUNC_SPI);
    gpio_set_function(MT6816_SCK, GPIO_FUNC_SPI);
    gpio_set_function(MT6816_MOSI, GPIO_FUNC_SPI);
    gpio_init(MT6816_CS);
    gpio_set_dir(MT6816_CS, GPIO_OUT);
    gpio_put(MT6816_CS, 1);
    
    // ------------------------------- 同时开启ADC和定时器 ------------------------------- 
    uint32_t mask = (1<<pwm_gpio_to_slice_num(PWM_A1)) | (1<<pwm_gpio_to_slice_num(PWM_B1));
    uint32_t save = save_and_disable_interrupts(); // 关闭中断
    adc_run(true);                                 // 开启ADC
    pwm_set_mask_enabled(mask);                    // 开启PWM，两个通道同步运行
    restore_interrupts(save);
    // ------------------------------- foc控制循环 ------------------------------- 
    foc_ctrl.foc_controller_ready = true;
    foc_loop(); 
    // ------------------------------- 故障模式 ------------------------------- 
    irq_set_enabled(DMA_IRQ_0, false);
    irq_set_enabled(PWM_IRQ_WRAP, false);
    // 如果存在任何异常，关闭所有PWM输出
    pwm_set_gpio_level(PWM_A1, 0);
    pwm_set_gpio_level(PWM_A2, 0);
    pwm_set_gpio_level(PWM_B1, 0);
    pwm_set_gpio_level(PWM_B2, 0);
    if(foc_ctrl.mode >= FIRST_PROTECT_MODE){
        int error_code_num = foc_ctrl.mode - FIRST_PROTECT_MODE + 1;
        error_code(error_code_num);
    }
}

