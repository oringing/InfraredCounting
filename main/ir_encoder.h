#ifndef IR_ENCODER_H_
#define IR_ENCODER_H_

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// 硬件配置宏定义（集中管理）
#define IR_TX_DATA       GPIO_NUM_18       // 红外发射引脚
#define IR_RX_OUT       GPIO_NUM_17       // 红外接收引脚
#define BOOT_PIN         GPIO_NUM_0        // BOOT按键引脚（启动/停止控制）
#define DEFAULT_IR_FREQ  1                 // 初始发射频率（Hz，每秒切换一次）
#define IR_TX_CHANNEL    LEDC_CHANNEL_5    // PWM通道
#define IR_TX_TIMER      LEDC_TIMER_3      // PWM定时器
#define WHEEL_DIAMETER_DEFAULT 0.066       // 默认车轮直径（米）,也就是66 mm
#define ENCODER_SLOTS    20                // 编码盘镂空槽数量（每转一圈产生20个脉冲）
#define PI               3.14159f          // π常量

class IREncoder {
private:
    // 硬件参数（使用宏定义初始化）
    gpio_num_t tx_pin_;           
    gpio_num_t rx_pin_;           
    uint32_t ir_freq_;            // 发射频率（Hz）
    ledc_channel_t tx_channel_;   
    ledc_timer_t tx_timer_;       

    // 计数与控制参数
    float wheel_diameter_;        
    float pulse_per_meter_;       
    volatile int pulse_count_;    
    SemaphoreHandle_t pulse_sem_; 
    volatile bool is_transmitting_; // 发射状态：true-启动，false-停止
    volatile bool has_received_;    // 接收标志（用于日志输出）
    SemaphoreHandle_t boot_sem_;    // 按键同步信号量

    // 静态中断处理函数
    static void IRAM_ATTR rx_isr_handler(void* arg);
    static void IRAM_ATTR boot_isr_handler(void* arg);

    // 发射控制任务
    static void transmit_task(void* arg);

public:
    // 构造函数（使用宏定义默认参数）
    IREncoder(float wheel_diameter = WHEEL_DIAMETER_DEFAULT, 
              gpio_num_t tx_pin = IR_TX_DATA, 
              gpio_num_t rx_pin = IR_RX_OUT, 
              uint32_t ir_freq = DEFAULT_IR_FREQ);
    
    esp_err_t init();

    // 脉冲与距离操作
    int get_pulse_count();
    void reset_pulse_count();
    float get_distance();
    void set_wheel_diameter(float diameter);

    // 发射控制
    void toggle_transmit();       // 切换发射状态（启动/停止）
    bool is_transmitting();       // 获取当前发射状态

    // 中断回调处理
    void handle_pulse();          // 处理接收脉冲
    void handle_boot_press();     // 处理BOOT键按下

    // 接收状态检查（用于日志输出）
    bool has_received();
    void reset_received_flag();
    
    // 信号量访问方法
    SemaphoreHandle_t get_boot_semaphore();  // 获取BOOT键信号量句柄

    // 新增：获取上次接收时间
    TickType_t get_last_receive_time();
};

#endif  // IR_ENCODER_H_