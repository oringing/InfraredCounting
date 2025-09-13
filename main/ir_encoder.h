#ifndef IR_ENCODER_H_
#define IR_ENCODER_H_

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "time.h"
#include "sys/time.h"
#include "driver/timer.h"

// 硬件配置宏定义（集中管理）
#define IR_RX_LEFT       GPIO_NUM_17       // 左轮红外接收引脚（OUT）
#define IR_RX_RIGHT      GPIO_NUM_18       // 右轮红外接收引脚（OUT）
#define BOOT_PIN         GPIO_NUM_0        // BOOT按键引脚（启动/停止控制）
#define WHEEL_DIAMETER_DEFAULT 0.066       // 默认车轮直径（米）,也就是66 mm
#define ENCODER_SLOTS    20                // 编码盘镂空槽数量（每转一圈产生20个脉冲）
#define PI               3.14159f          // π常量
#define LOG_PULSE_INTERVAL 5               // 日志打印间隔（每N个脉冲打印一次）
#define NO_SIGNAL_TIMEOUT 5000             // 无信号超时时间（毫秒）

class WheelEncoder {
private:
    // GPIO中断处理函数
    static void IRAM_ATTR IR_ISR(void* arg);
    
    // 静态成员变量用于存储实例指针
    static WheelEncoder* left_instance_;
    static WheelEncoder* right_instance_;

    // 硬件参数
    gpio_num_t rx_pin_;           // 接收引脚

    // 计数与控制参数
    float wheel_diameter_;
    volatile int pulse_count_;    
    SemaphoreHandle_t pulse_sem_; 
    volatile bool has_received_;    // 接收标志
    SemaphoreHandle_t boot_sem_;    // 按键同步信号量

    // 时间戳
    TickType_t last_receive_time_;  
    TickType_t transmit_start_time_;  // 保留为起始时间标记
    struct timeval transmit_start_real_time_;

    // 信号检测相关变量
    volatile bool last_signal_state_;     // 上次信号状态
    volatile uint32_t detection_counter_; // 检测计数器

    // 静态中断处理函数
    static void IRAM_ATTR boot_isr_handler(void* arg);
    // 已注释掉定时器中断处理函数
    // static bool IRAM_ATTR detection_timer_isr(void* arg);

    // 接收任务
    static void receive_task(void* arg);


public:
    // 构造函数
    WheelEncoder(float wheel_diameter = WHEEL_DIAMETER_DEFAULT, 
              gpio_num_t rx_pin = IR_RX_LEFT);

    esp_err_t init();

    // 脉冲与距离操作
    int get_pulse_count();
    void reset_pulse_count();
    float get_distance();
    void set_wheel_diameter(float diameter);

    // 中断回调处理
    void handle_boot_press();     // 处理BOOT键按下
    // 已注释掉定时器检测处理函数
    // void handle_signal_detection(); // 处理信号检测

    // 接收状态检查
    bool has_received();
    bool is_receiving();  // 添加缺失的声明
    void reset_received_flag();

    // 信号量访问方法
    SemaphoreHandle_t get_boot_semaphore();

    // 获取时间信息
    TickType_t get_last_receive_time();
    TickType_t get_transmit_start_time();
    struct timeval get_transmit_start_real_time();
    
    // 静态方法获取实例
    static WheelEncoder* get_left_instance();
    static WheelEncoder* get_right_instance();
};

// 距离控制函数
void move_distance(float distance, bool forward=true);
void move_forward(float distance);  // 前进指定距离
void move_backward(float distance); // 后退指定距离

#endif  // IR_ENCODER_H_