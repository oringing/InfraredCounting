#include "ir_encoder.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "time.h"
#include "sys/time.h"

static const char* TAG = "WHEEL_ENCODER";

// 静态成员变量初始化
WheelEncoder* WheelEncoder::left_instance_ = nullptr;
WheelEncoder* WheelEncoder::right_instance_ = nullptr;

// 构造函数初始化
WheelEncoder::WheelEncoder(float wheel_diameter, gpio_num_t rx_pin)
    : rx_pin_(rx_pin), wheel_diameter_(wheel_diameter),
      pulse_count_(0), has_received_(false), last_receive_time_(0),
      transmit_start_time_(0), last_signal_state_(false), detection_counter_(0) {
    pulse_sem_ = xSemaphoreCreateBinary();
    boot_sem_ = xSemaphoreCreateBinary();
    transmit_start_real_time_.tv_sec = 0;
    transmit_start_real_time_.tv_usec = 0;
    
    // 根据引脚设置实例
    if (rx_pin == IR_RX_LEFT) {
        left_instance_ = this;
    } else if (rx_pin == IR_RX_RIGHT) {
        right_instance_ = this;
    }
}


// 初始化硬件
esp_err_t WheelEncoder::init() {
    // 1. 配置接收引脚（输入+上拉，下降沿中断）
    gpio_config_t rx_conf = {
        .pin_bit_mask = (1ULL << rx_pin_),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // 设置为下降沿中断
    };
    gpio_config(&rx_conf);

    // 2. 配置BOOT键（输入+中断）
    gpio_config_t boot_conf = {
        .pin_bit_mask = (1ULL << BOOT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&boot_conf);
    
    // 安装GPIO中断服务
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BOOT_PIN, boot_isr_handler, this);
    
    // 注册红外接收引脚的中断处理函数
    gpio_isr_handler_add(rx_pin_, IR_ISR, this);

    // 3. 已注释掉定时器配置
    /*
    // 配置检测定时器（每5ms触发一次）
    timer_config_t detection_timer_config;
    detection_timer_config.alarm_en = TIMER_ALARM_EN;
    detection_timer_config.counter_en = TIMER_PAUSE;
    detection_timer_config.intr_type = TIMER_INTR_LEVEL;
    detection_timer_config.counter_dir = TIMER_COUNT_UP;
    detection_timer_config.auto_reload = TIMER_AUTORELOAD_EN;
    detection_timer_config.divider = 80;   // 1MHz计数频率
    detection_timer_config.clk_src = TIMER_SRC_CLK_DEFAULT;

    timer_init(DETECTION_TIMER_GROUP, DETECTION_TIMER_IDX, &detection_timer_config);
    timer_set_alarm_value(DETECTION_TIMER_GROUP, DETECTION_TIMER_IDX, 5000); // 5ms
    timer_isr_callback_add(DETECTION_TIMER_GROUP, DETECTION_TIMER_IDX, detection_timer_isr, this, ESP_INTR_FLAG_IRAM);

    timer_pause(DETECTION_TIMER_GROUP, DETECTION_TIMER_IDX);
    timer_set_counter_value(DETECTION_TIMER_GROUP, DETECTION_TIMER_IDX, 0);
    */

    // 4. 创建接收任务
    xTaskCreate(receive_task, "wheel_receive_task", 4096, this, 4, NULL);

    ESP_LOGI(TAG, "接收模式初始化完成，引脚: %d", rx_pin_);
    return ESP_OK;
}


// 接收任务：持续运行，等待信号变化
void WheelEncoder::receive_task(void* arg) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(arg);
    while (1) {
        // 等待脉冲信号
        if (xSemaphoreTake(encoder->pulse_sem_, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // 处理接收到的脉冲信号
            // 日志输出在main.cc中处理
        }
    }
}



// 新增：判断是否正在接收（始终为true，因为只做接收）
bool WheelEncoder::is_receiving() {
    return true;
}


// GPIO中断处理函数
void IRAM_ATTR WheelEncoder::IR_ISR(void* arg) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(arg);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 直接增加计数器（因为中断已经是下降沿触发）
    encoder->pulse_count_ = encoder->pulse_count_ + 1;
    encoder->has_received_ = true;
    encoder->last_receive_time_ = xTaskGetTickCountFromISR();
    xSemaphoreGiveFromISR(encoder->pulse_sem_, &xHigherPriorityTaskWoken);
    
    // 如果需要，请求上下文切换
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


/*
// 已注释掉定时器中断处理函数
// 新增：信号检测定时器中断处理
bool IRAM_ATTR WheelEncoder::detection_timer_isr(void* arg) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(arg);
    encoder->handle_signal_detection();
    timer_group_clr_intr_status_in_isr(DETECTION_TIMER_GROUP, DETECTION_TIMER_IDX);
    timer_group_enable_alarm_in_isr(DETECTION_TIMER_GROUP, DETECTION_TIMER_IDX);
    return true;
}
*/

/*
// 已注释掉定时器检测处理函数
// 处理信号检测逻辑
void WheelEncoder::handle_signal_detection() {
    bool current_signal = gpio_get_level(rx_pin_);
    detection_counter_ = detection_counter_ + 1;  // 替换++操作以避免警告

    if (current_signal != last_signal_state_) {
        last_signal_state_ = current_signal;

        if (!current_signal) {
            // 从高到低：表示遮挡发生（脉冲上升沿）
            pulse_count_ = pulse_count_ + 1;  // 替换++操作以避免警告
            has_received_ = true;
            last_receive_time_ = xTaskGetTickCountFromISR();
            xSemaphoreGiveFromISR(pulse_sem_, NULL);
        }
    }
}
*/


// BOOT键中断处理
void IRAM_ATTR WheelEncoder::boot_isr_handler(void* arg) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(arg);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 简单的软件去抖动
    esp_rom_delay_us(1000);
    xSemaphoreGiveFromISR(encoder->boot_sem_, &xHigherPriorityTaskWoken);
    
    // 如果需要，请求上下文切换
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

//boot键按下处理
void WheelEncoder::handle_boot_press() {
    reset_pulse_count();
    transmit_start_time_ = xTaskGetTickCount();
    gettimeofday(&transmit_start_real_time_, NULL);
    ESP_LOGI(TAG, "计数已重置");
}

// 其他函数实现
int WheelEncoder::get_pulse_count() {
    return pulse_count_;
}

void WheelEncoder::reset_pulse_count() {
    pulse_count_ = 0;
}

float WheelEncoder::get_distance() {
    return (float)pulse_count_ * PI * wheel_diameter_ / ENCODER_SLOTS;
}

void WheelEncoder::set_wheel_diameter(float diameter) {
    wheel_diameter_ = diameter;
}

bool WheelEncoder::has_received() {
    return has_received_;
}

void WheelEncoder::reset_received_flag() {
    has_received_ = false;
}

TickType_t WheelEncoder::get_last_receive_time() {
    return last_receive_time_;
}

TickType_t WheelEncoder::get_transmit_start_time() {
    return transmit_start_time_;
}

struct timeval WheelEncoder::get_transmit_start_real_time() {
    return transmit_start_real_time_;
}

SemaphoreHandle_t WheelEncoder::get_boot_semaphore() {
    return boot_sem_;
}

WheelEncoder* WheelEncoder::get_left_instance() {
    return left_instance_;
}

WheelEncoder* WheelEncoder::get_right_instance() {
    return right_instance_;
}

// 距离控制函数实现
void move_distance(float distance, bool forward) {
    // 这里应该实现闭环控制逻辑，目前作为开环控制示例
    ESP_LOGI(TAG, "%s 移动 %.2f 米", forward ? "前进" : "后退", distance);
    // 实际应用中这里应该控制电机
}

void move_forward(float distance) {
    move_distance(distance, true);
}

void move_backward(float distance) {
    move_distance(distance, false);
}