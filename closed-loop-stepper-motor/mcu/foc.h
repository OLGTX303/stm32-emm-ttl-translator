#ifndef FOC_H
#define FOC_H

#define PWM_TOP            2499    // 125Mz / 2 /(2499+1) = 25KHz
#define PWM_MAX            2400    // 考虑到100%占空比时自举升压电容无法充电，限制最大占空比
#define PWM_MIN            16      // 驱动芯片无法响应过短的脉冲，限制最小占空比
#define MAX_CURRENT        ((int)(2.5/3.3*2048+0.5))  // 最大允许电流2.5A, 1.8A * sqrt(2)
#define LOW_SPEED_CURRENT  0.3f  // 静止和低速时保持正的id，而不是id=0，降低噪声
#define LED_ON_CURRENT     ((int)(0.5/3.3*2048+0.5))  // 超过0.5A LED亮起

// 原始速度数据到电角速度（rad/s）的换算系数
// speed是最近SPEED_BUFFER_SIZE*0.2ms multiple_angle的增量
// 如果要换算为RPM： speed * (60000ms / SPEED_BUFFER_SIZE*0.2ms)) / 16384 
#define SPEED_BUFFER_SIZE  4
#define SPEED_TO_RPM       (float)((60000.0 / (SPEED_BUFFER_SIZE*0.2)) / 16384.0)
#define RPM_TO_SPEED       (1.0f/SPEED_TO_RPM)
#define SPEED_TO_W         (float)((60000.0 / (SPEED_BUFFER_SIZE*0.2)) / 16384.0 * (2.0 * M_PI * 50.0) / 60.0)
// 最大速度，这个是位置环的限制，如果只用速度环或者电流环，可用设定到更高的转速
#define MAX_SPEED          ((int)(1600.0 * RPM_TO_SPEED + 0.5))
// 堵转检测阈值，误差超过2步，就认为出现堵转
#define BLOCK_DETECT_VALUE ((int)(2.0f * ENCODER_MAX / TOTAL_SECTOR + 0.5f))

#define TOTAL_SECTOR       200     // 一圈步数
#define ENCODER_MAX        16384   // 编码器最大值
#define CAPTURE_DEPTH      20      // ADC DMA buffer长度，A相电流、B相电流各占1/2
#define VBUS_FACTOR        (float)((47.0f + 5.1f) / 5.1f * 3.3f / (1 << 12))

typedef enum {
    MODE_VOLATGE = 0, // angle,volatage根据控制输出电压
    MODE_CURRENT = 1,    // 根据iq_set、id_set控制输出电流
    MODE_SPEED = 2,
    MODE_POSITION = 3,
    MODE_POSITION_WITH_TRAPEZOID = 4,  // 梯形位置控制
    MODE_OVER_VOLTAGE_PROTECT = 5,     // 过压保护         LED模式：短短短长
    MODE_UNDER_VOLTAGE_PROTECT = 6,    // 欠压保护         LED模式：短短长短
    MODE_OVER_CURRENT_PROTECT = 7,     // 过流保护         LED模式：短短长长
    MODE_OVER_SPEED_PROTECT = 8,       // 过速保护         LED模式：短长短短
    MODE_OVER_TEMPERATURE_PROTECT = 9, // 过热保护         LED模式：短长短长
    MODE_MT6816_FAIL_PROTECT = 10,     // 角度传感器损坏保护 LED模式：短长长短
} foc_mode;
#define FIRST_PROTECT_MODE MODE_OVER_VOLTAGE_PROTECT
#define PROTECT_OVER_VOLTAGE     (int)(30.0/VBUS_FACTOR + 0.5)      // 30V
#define PROTECT_UNDER_VOLTAGE    (int)(18.0/VBUS_FACTOR + 0.5)      // 18V
#define PROTECT_OVER_CURRENT     ((int)(3.2/3.3*2048+0.5))          // 3.2A
#define PROTECT_OVER_SPEED       (int)(4000.0*RPM_TO_SPEED + 0.5)   // 4000RPM
#define PROTECT_OVER_TEMPERATURE ((int)(4096.0/3.3*(0.706-0.001721*(85.0-27.0)))+0.5)  // 85℃
#define PROTECT_MT6816_ERROR_COUNT 4

typedef struct {
    // 角度（位置）类
    int mt6816_angle_read;    // 0-16383       磁编码器原始数据
    int machanic_angle;       // 0-16383       校准后的编码器数据，单圈
    int multiple_angle_read;  // 2^31-2^31-1   机械角度，多圈，每圈16384
    int multiple_angle_target;// 2^31-2^31-1   机械角度目标，多圈，用于位置环
    int electro_angle;        // 0-1023        电角度
    int pos_kp;
    int pos_kd;
    int mt6816_fail_count;
    // 速度类
    int speed_read;           // 当前转速，用于速度环和弱磁控制
    int speed_target;         // 转速目标
    int speed_kp;
    int speed_ki;
    bool update_speed;
    // 电压类
    int vd;                   // ±2400        d轴电压
    int vq;                   // ±2400        q轴电压
    int vd_ff;                // ±2400        d轴前馈电压,供调试使用
    int vq_ff;                // ±2400        q轴前馈电压,供调试使用
    int v_alpha;              // ±2400        绕组A电压
    int reverse_v_alpha;      // 
    int v_beta;               // ±2400        绕组B电压
    // int sqrt_vd_vq;
    int power_voltage_raw;
    int power_voltage_filter;
    // 电流类
    int iq_target;            // ±2048        q轴目标电流 
    int id_target;            // ±2048        d轴目标电流
    int i_offset_alpha;       // 2048左右      A绕组电流偏移量
    int i_offset_beta;        // 2048左右      B绕组电流偏移量
    int i_alpha;              // ±2048        A绕组电流
    int i_beta;               // ±2048        B绕组电流
    int id;                   // ±2048        q轴电流，决定磁通
    int iq;                   // ±2048        q轴电流，决定转矩
    int curr_kp;              // 
    int curr_ki;              // 
    float low_speed_id_current;
    // 控制类
    foc_mode mode;            // 0-2          工作模式
    bool foc_controller_ready;// 0 - 1        初始化完成标志
    bool enable_ff;           // 是否打开电流环前馈
    bool trapezoid_running;
    // 温度类
    int temperature_raw;
    // 调试类
    int fill_debug_buf;                  // 要抓取的数据的数量
    bool debug_div_5;                    // true表示每5个控制周期据抓1组数据
    volatile int *debug_date_address[5]; // 要抓取的数据的地址
    int zero;                            // 恒定为0，不要修改
    // 电流环前馈，以及弱磁
    int w_Lq;
    int w_Ld;
    int w_psi;
    // 弱磁控制
    float psi; 
    float Ld;
    float Lq;
    float Rs;
    // 运动控制部分
    bool stop_when_block;         // ==true时，堵转时会停在堵转时的位置
    bool trap_run;                // ==true时，执行运动控制，否则即使fifo有数据也不会执行
    bool block;                   // ==true时，发生堵转问题
    int max_error;                // 从上次读取到现在的最大位置误差
}foc_ctrl_struct;

#define FIFO_DEPTH 512
#define FIFO_MASK (FIFO_DEPTH - 1)
typedef struct {
    // FIFO部分
    int32_t buffer[FIFO_DEPTH];
    uint8_t current_buffer[FIFO_DEPTH];
    uint32_t read_index;
    uint32_t write_index;
    uint32_t count;
    spin_lock_t *lock;
} fifo_t;

extern volatile foc_ctrl_struct foc_ctrl;
extern fifo_t foc_fifo;
extern int16_t calibration[TOTAL_SECTOR+1];

// 调试用，抓取到的数据
#define DEBUG_BUF_LEN      4096    // 80K debug buffer
extern int debug_data_buf[DEBUG_BUF_LEN][5];


void core1_entry(void);
void fifo_init(fifo_t *f);
void fifo_clear(fifo_t *f);
int decompress_and_write_fifo(const uint8_t* compressed, uint32_t compressed_size, fifo_t *fifo);
uint32_t fifo_count(fifo_t *f);
#endif