#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
// 待测试的中位数函数
static int get_adc_average(uint16_t *buf, int offset) {
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

// 测试辅助函数：标准中位数计算方法
static int calculate_median(uint16_t values[5]) {
    uint16_t sorted[5];
    memcpy(sorted, values, sizeof(sorted));
    
    // 冒泡排序
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4 - i; j++) {
            if (sorted[j] > sorted[j+1]) {
                uint16_t temp = sorted[j];
                sorted[j] = sorted[j+1];
                sorted[j+1] = temp;
            }
        }
    }
    return (int)sorted[2];
}

// 测试用例执行器
void run_test_case(uint16_t test_case[5], int case_num) {
    uint16_t buffer[20] = {0}; // 模拟ADC缓冲区
    
    // 按间隔4存储测试数据
    for (int i = 0; i < 5; i++) {
        buffer[i*4] = test_case[i];
    }

    int expected = calculate_median(test_case);
    int actual = get_adc_average(buffer, 0);
    
    if (actual != expected) {
        printf("测试失败 #%d\n", case_num);
        printf("输入: [%u, %u, %u, %u, %u]\n", 
               test_case[0], test_case[1], test_case[2],
               test_case[3], test_case[4]);
        printf("预期: %d, 实际: %d\n", expected, actual);
        exit(1);
    }
}

int main(void){
    srand(time(NULL));
    int passed = 0;

    // 测试模式选择
    enum TestMode { BOUNDARY, ORDERED, DUPLICATES, RANDOM, MODE_COUNT };
    
    for (int mode = 0; mode < MODE_COUNT; mode++) {
        printf("正在执行模式 %d/%d...\n", mode+1, MODE_COUNT);
        
        for (int i = 0; i < 1000; i++) { // 每种模式1000次测试
            uint16_t test_case[5];
            
            switch (mode) {
                case BOUNDARY:  // 边界值测试
                    test_case[0] = 0;
                    test_case[1] = 0;
                    test_case[2] = UINT16_MAX;
                    test_case[3] = rand() % UINT16_MAX;
                    test_case[4] = UINT16_MAX;
                    break;
                    
                case ORDERED:  // 有序序列测试
                    for (int j = 0; j < 5; j++) {
                        test_case[j] = (i % 2) ? j * 1000 : UINT16_MAX - j * 1000;
                    }
                    break;
                    
                case DUPLICATES:  // 重复值测试
                    {
                        uint16_t base = rand() % 50000;
                        for (int j = 0; j < 5; j++) {
                            test_case[j] = base + (rand() % 3);
                        }
                    }
                    break;
                    
                default:  // 随机测试
                    for (int j = 0; j < 5; j++) {
                        test_case[j] = rand() % UINT16_MAX;
                    }
            }
            
            run_test_case(test_case, passed + 1);
            passed++;
        }
    }

    printf("所有%d个测试用例通过！\n", passed);
    return 0;
}