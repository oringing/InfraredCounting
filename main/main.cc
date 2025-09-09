#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ir_encoder.h"
#include "esp_log.h"

static const char* TAG = "MAIN";

// 按键处理任务（处理BOOT键中断）
void boot_task(void* pvParameters) {
    IREncoder* encoder = static_cast<IREncoder*>(pvParameters);
    while (1) {
        if (xSemaphoreTake(encoder->get_boot_semaphore(), portMAX_DELAY) == pdTRUE) {
            encoder->handle_boot_press();  // 切换发射状态
        }
    }
}

// 测试任务（打印状态+接收日志）
void test_task(void* pvParameters) {
    IREncoder* encoder = static_cast<IREncoder*>(pvParameters);
    
    // 记录上次接收时间
    TickType_t last_receive_time = 0;
    bool log_enabled = false;  // 日志是否启用
    
    while (1) {
        // 检查是否接收到红外信号
        if (encoder->has_received()) {
            // 更新上次接收时间
            last_receive_time = encoder->get_last_receive_time();
            
            // 启用日志输出
            log_enabled = true;
            
            // 打印脉冲数和距离
            int pulse = encoder->get_pulse_count();
            float distance = encoder->get_distance();
            ESP_LOGI(TAG, "脉冲数: %d, 距离: %.2f米", pulse, distance);
            
            // 重置接收标志
            encoder->reset_received_flag();
        }
        
        // 检查是否需要打印未接收到信号的提示
        if (log_enabled && (xTaskGetTickCount() - last_receive_time) > pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG, "未接收到红外信号...");
            log_enabled = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "红外计数模块测试程序启动");

    // 创建红外编码器实例（参数通过宏定义，无需硬编码）
    IREncoder encoder;
    
    // 初始化硬件
    esp_err_t ret = encoder.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化失败，程序退出");
        return;
    }

    // 创建测试任务和按键处理任务
    xTaskCreate(test_task, "test_task", 4096, &encoder, 5, NULL);
    xTaskCreate(boot_task, "boot_task", 2048, &encoder, 5, NULL);

    // 主循环
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}