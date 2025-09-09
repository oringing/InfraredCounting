#include "ir_encoder.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char* TAG = "IR_ENCODER";

// 构造函数初始化
IREncoder::IREncoder(float wheel_diameter, gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t ir_freq)
    : tx_pin_(tx_pin), rx_pin_(rx_pin), ir_freq_(ir_freq),
      tx_channel_(IR_TX_CHANNEL), tx_timer_(IR_TX_TIMER),
      wheel_diameter_(wheel_diameter), pulse_count_(0), is_transmitting_(false),
      has_received_(false) {
    pulse_sem_ = xSemaphoreCreateBinary();
    boot_sem_ = xSemaphoreCreateBinary();
}

// 初始化硬件
esp_err_t IREncoder::init() {
    // 1. 配置发射引脚（输出模式）
    gpio_config_t tx_conf = {
        .pin_bit_mask = (1ULL << tx_pin_),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&tx_conf);
    gpio_set_level(tx_pin_, 0);  // 初始低电平

    // 2. 配置接收引脚（输入+中断）
    gpio_config_t rx_conf = {
        .pin_bit_mask = (1ULL << rx_pin_),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE  // 上升沿触发（检测高电平）
    };
    gpio_config(&rx_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(rx_pin_, rx_isr_handler, this);

    // 3. 配置BOOT键（输入+中断）
    gpio_config_t boot_conf = {
        .pin_bit_mask = (1ULL << BOOT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // 内部上拉，按下时为低电平
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // 下降沿触发（按下时）
    };
    gpio_config(&boot_conf);
    gpio_isr_handler_add(BOOT_PIN, boot_isr_handler, this);

    // 4. 创建发射控制任务
    xTaskCreate(transmit_task, "ir_transmit_task", 2048, this, 4, NULL);

    ESP_LOGI(TAG, "硬件初始化完成");
    return ESP_OK;
}

// 发射任务：周期性切换高低电平（1秒低→1秒高）
void IREncoder::transmit_task(void* arg) {
    IREncoder* encoder = static_cast<IREncoder*>(arg);
    while (1) {
        if (encoder->is_transmitting_) {
            // 低电平1秒
            gpio_set_level(encoder->tx_pin_, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            // 高电平3.3V 1秒
            gpio_set_level(encoder->tx_pin_, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // 停止发射时保持低电平
            gpio_set_level(encoder->tx_pin_, 0);
            vTaskDelay(pdMS_TO_TICKS(100));  // 降低CPU占用
        }
    }
}

// 接收中断处理
void IRAM_ATTR IREncoder::rx_isr_handler(void* arg) {
    IREncoder* encoder = static_cast<IREncoder*>(arg);
    encoder->handle_pulse();
}

// 处理接收脉冲（设置接收标志）
void IREncoder::handle_pulse() {
    pulse_count_++;
    has_received_ = true;  // 标记收到信号
    xSemaphoreGiveFromISR(pulse_sem_, NULL);
}

// BOOT键中断处理
void IRAM_ATTR IREncoder::boot_isr_handler(void* arg) {
    IREncoder* encoder = static_cast<IREncoder*>(arg);
    xSemaphoreGiveFromISR(encoder->boot_sem_, NULL);
}

// 处理BOOT键按下（切换发射状态）
void IREncoder::handle_boot_press() {
    toggle_transmit();
    ESP_LOGI(TAG, "发射状态切换：%s", is_transmitting_ ? "开启" : "关闭");
}

// 切换发射状态
void IREncoder::toggle_transmit() {
    is_transmitting_ = !is_transmitting_;
}

// 获取发射状态
bool IREncoder::is_transmitting() {
    return is_transmitting_;
}

// 接收状态检查与重置
bool IREncoder::has_received() {
    return has_received_;
}

void IREncoder::reset_received_flag() {
    has_received_ = false;
}

// 其他函数实现（脉冲计数、距离计算等）
int IREncoder::get_pulse_count() {
    return pulse_count_;
}

void IREncoder::reset_pulse_count() {
    pulse_count_ = 0;
}

float IREncoder::get_distance() {
   // 根据公式计算距离: L = C × πD / N
    // C: 脉冲计数, D: 车轮直径, N: 编码盘镂空槽数量
    return (float)pulse_count_ * PI * wheel_diameter_ / ENCODER_SLOTS;
}

void IREncoder::set_wheel_diameter(float diameter) {
    wheel_diameter_ = diameter;
}

// 获取BOOT键信号量句柄
SemaphoreHandle_t IREncoder::get_boot_semaphore() {
    return boot_sem_;
}

// 获取上次接收时间
TickType_t IREncoder::get_last_receive_time() {
    return xTaskGetTickCount();
}