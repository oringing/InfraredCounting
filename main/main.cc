#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ir_encoder.h"
#include "esp_log.h"
#include "time.h"
#include "sys/time.h"

static const char* TAG = "MAIN";

// 按键处理任务
void boot_task(void* pvParameters) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(pvParameters);
    while (1) {
        if (xSemaphoreTake(encoder->get_boot_semaphore(), portMAX_DELAY) == pdTRUE) {
            encoder->handle_boot_press();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// 测试任务（打印状态+接收日志）
void test_task(void* pvParameters) {
    WheelEncoder* left_encoder = WheelEncoder::get_left_instance();
    WheelEncoder* right_encoder = WheelEncoder::get_right_instance();

    TickType_t last_left_receive_time = 0;
    TickType_t last_right_receive_time = 0;
    bool left_log_enabled = false;
    bool right_log_enabled = false;
    int last_logged_left_count = 0;
    int last_logged_right_count = 0;

    while (1) {
        // 处理左轮编码器
        if (left_encoder && left_encoder->has_received()) {
            last_left_receive_time = left_encoder->get_last_receive_time();
            left_log_enabled = true;

            int pulse_count = left_encoder->get_pulse_count();
            if (pulse_count >= last_logged_left_count + LOG_PULSE_INTERVAL) {
                TickType_t current_time = xTaskGetTickCount();
                TickType_t start_time = left_encoder->get_transmit_start_time();
                uint32_t relative_time_ms = (current_time - start_time) * portTICK_PERIOD_MS;

                ESP_LOGI(TAG, "脉冲被遮挡: %d 次(左轮--)  |  时间: %lu.%03lu ms(左--)  |  距离: %.2f m(左--)",
                         pulse_count,
                         relative_time_ms / 1000,
                         relative_time_ms % 1000,
                         left_encoder->get_distance());

                last_logged_left_count = pulse_count - (pulse_count % LOG_PULSE_INTERVAL);
            }

            left_encoder->reset_received_flag();
        }

        // 处理右轮编码器
        if (right_encoder && right_encoder->has_received()) {
            last_right_receive_time = right_encoder->get_last_receive_time();
            right_log_enabled = true;

            int pulse_count = right_encoder->get_pulse_count();
            if (pulse_count >= last_logged_right_count + LOG_PULSE_INTERVAL) {
                TickType_t current_time = xTaskGetTickCount();
                TickType_t start_time = right_encoder->get_transmit_start_time();
                uint32_t relative_time_ms = (current_time - start_time) * portTICK_PERIOD_MS;

                ESP_LOGI(TAG, "脉冲被遮挡: %d 次(--右轮)  |  时间: %lu.%03lu ms(--右)  |  距离: %.2f m(--右)",
                         pulse_count,
                         relative_time_ms / 1000,
                         relative_time_ms % 1000,
                         right_encoder->get_distance());

                last_logged_right_count = pulse_count - (pulse_count % LOG_PULSE_INTERVAL);
            }

            right_encoder->reset_received_flag();
        }

        // 检查左轮超时
        if (left_log_enabled && left_encoder && 
            (xTaskGetTickCount() - last_left_receive_time) > pdMS_TO_TICKS(NO_SIGNAL_TIMEOUT)) {
            ESP_LOGI(TAG, "左轮未检测到遮挡事件，当前计数：%d", left_encoder->get_pulse_count());
            left_log_enabled = false;
        }

        // 检查右轮超时
        if (right_log_enabled && right_encoder && 
            (xTaskGetTickCount() - last_right_receive_time) > pdMS_TO_TICKS(NO_SIGNAL_TIMEOUT)) {
            ESP_LOGI(TAG, "右轮未检测到遮挡事件，当前计数：%d", right_encoder->get_pulse_count());
            right_log_enabled = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "红外测速传感器测试程序启动");

    static WheelEncoder left_encoder(WHEEL_DIAMETER_DEFAULT, IR_RX_LEFT);
    static WheelEncoder right_encoder(WHEEL_DIAMETER_DEFAULT, IR_RX_RIGHT);

    esp_err_t ret_left = left_encoder.init();
    esp_err_t ret_right = right_encoder.init();
    
    if (ret_left != ESP_OK || ret_right != ESP_OK) {
        ESP_LOGE(TAG, "初始化失败，程序退出");
        return;
    }

    xTaskCreate(test_task, "test_task", 4096, NULL, 5, NULL);
    xTaskCreate(boot_task, "boot_task", 4096, &left_encoder, 5, NULL);

    // 测试距离控制函数
    move_forward(1.0);
    move_backward(0.5);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}