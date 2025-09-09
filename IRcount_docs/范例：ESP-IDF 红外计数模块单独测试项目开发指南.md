# 范例：ESP-IDF 红外计数模块单独测试项目开发指南（含项目模板选择、代码实现与硬件测试）
### ESP-IDF 项目模板选择建议
![ESP-IDF 项目模板选择](ESP-IDF项目模板选择.png)
推荐选择 **`template-app`** 模板，原因如下：  
1. 现有仓库（`xiaozhi-esp32-motor`）是基于纯 ESP-IDF 框架开发的（使用 `esp_log.h`、`freertos` 等原生组件，无 Arduino 依赖），`template-app` 作为最基础的通用模板，其项目结构（`main` 目录、`CMakeLists.txt` 组织方式）与现有仓库完全兼容，后续代码迁移和封装更顺畅。  
2. 无需引入 Arduino 库（`arduino-as-component` 可能增加冗余依赖），也无需示例模板（`fibonacci-app`）或测试框架（`unity-app`），专注于核心功能开发即可。  


以下是基于ESP-IDF的红外计数模块单独测试项目实现步骤，全程讲解代码编写逻辑和库调用方法，帮助你掌握从0到1的开发测试流程：


### 一、新建ESP-IDF项目
1. 打开ESP-IDF Command Prompt，创建新项目：
```bash
idf.py create-project ir_encoder_test
cd ir_encoder_test
```
2. 项目结构说明（重点关注以下文件）：
```
ir_encoder_test/
├── CMakeLists.txt       # 项目配置
├── sdkconfig            # 编译配置（后续会自动生成）
└── main/
    ├── CMakeLists.txt   # 源文件配置
    └── app_main.cpp     # 主程序入口
```


### 二、编写头文件（ir_encoder.h）
在`main`文件夹下新建`ir_encoder.h`，定义红外编码器类（复用核心逻辑，简化测试版本）：
```cpp
#ifndef IR_ENCODER_H_
#define IR_ENCODER_H_

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class IREncoder {
private:
    // 硬件参数
    gpio_num_t tx_pin_;           // 红外发射引脚
    gpio_num_t rx_pin_;           // 红外接收引脚
    uint32_t ir_freq_;            // 红外频率（通常38kHz）
    ledc_channel_t tx_channel_;   // PWM通道（选未占用的，如LEDC_CHANNEL_5）
    ledc_timer_t tx_timer_;       // PWM定时器（如LEDC_TIMER_3）

    // 计数参数
    float wheel_diameter_;        // 车轮直径（米）
    float pulse_per_meter_;       // 每米脉冲数（预计算）
    volatile int pulse_count_;    // 脉冲计数（volatile确保中断/任务可见）
    SemaphoreHandle_t pulse_sem_; // 脉冲同步信号量

    // 静态中断处理函数（必须静态，才能注册为ISR）
    static void IRAM_ATTR rx_isr_handler(void* arg);

public:
    // 构造函数：初始化硬件参数
    IREncoder(float wheel_diameter, gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t ir_freq = 38000);
    
    // 初始化硬件（分离构造函数，便于错误处理）
    esp_err_t init();

    // 脉冲计数操作
    int get_pulse_count();
    void reset_pulse_count();

    // 距离计算
    float get_distance();         // 单位：米
    void set_wheel_diameter(float diameter);

    // 中断回调中调用的脉冲处理函数
    void handle_pulse();
};

#endif  // IR_ENCODER_H_
```


### 三、编写源文件（ir_encoder.cpp）
在`main`文件夹下新建`ir_encoder.cpp`，实现核心功能：
```cpp
#include "ir_encoder.h"
#include "esp_log.h"
#include <cmath>

// 日志标签（ESP-IDF标准用法，便于过滤日志）
static const char* TAG = "IR_ENCODER";

// 构造函数：初始化参数，不涉及硬件操作
IREncoder::IREncoder(float wheel_diameter, gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t ir_freq) 
    : wheel_diameter_(wheel_diameter), 
      tx_pin_(tx_pin), 
      rx_pin_(rx_pin), 
      ir_freq_(ir_freq),
      tx_channel_(LEDC_CHANNEL_5), 
      tx_timer_(LEDC_TIMER_3),
      pulse_count_(0), 
      pulse_sem_(nullptr) {
    // 预计算每米脉冲数：车轮周长=π×直径，假设编码盘每圈20脉冲
    pulse_per_meter_ = 20.0f / (M_PI * wheel_diameter_);
    ESP_LOGI(TAG, "每米脉冲数: %.1f", pulse_per_meter_);
}

// 初始化硬件：红外发射（PWM）+ 红外接收（中断）
esp_err_t IREncoder::init() {
    // 1. 初始化信号量（用于同步中断和任务）
    pulse_sem_ = xSemaphoreCreateBinary();
    if (!pulse_sem_) {
        ESP_LOGE(TAG, "信号量创建失败");
        return ESP_FAIL;
    }

    // 2. 配置红外发射（LEDC生成PWM）
    // 2.1 配置定时器
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,  // 低速模式（足够驱动红外）
        .duty_resolution = LEDC_TIMER_8_BIT,// 8位分辨率（占空比0-255）
        .timer_num = tx_timer_,             // 选择定时器
        .freq_hz = ir_freq_,                // 红外频率（38kHz）
        .clk_cfg = LEDC_AUTO_CLK            // 自动选择时钟源
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "定时器配置失败");
        return ESP_FAIL;
    }

    // 2.2 配置通道（绑定GPIO）
    ledc_channel_config_t ch_cfg = {
        .gpio_num = tx_pin_,                // 发射引脚
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = tx_channel_,             // 选择通道
        .timer_sel = tx_timer_,             // 绑定定时器
        .duty = 128,                        // 50%占空比（38kHz下有效）
        .hpoint = 0
    };
    if (ledc_channel_config(&ch_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "通道配置失败");
        return ESP_FAIL;
    }

    // 3. 配置红外接收（GPIO中断）
    gpio_config_t rx_config = {
        .pin_bit_mask = (1ULL << rx_pin_),  // 接收引脚
        .mode = GPIO_MODE_INPUT,            // 输入模式
        .pull_up_en = GPIO_PULLUP_ENABLE,   // 上拉（避免悬空）
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE      // 双边沿触发（检测上升/下降沿）
    };
    if (gpio_config(&rx_config) != ESP_OK) {
        ESP_LOGE(TAG, "接收引脚配置失败");
        return ESP_FAIL;
    }

    // 4. 注册中断服务
    // 4.1 安装ISR服务（全局仅需一次，测试项目可直接调用）
    if (gpio_install_isr_service(0) != ESP_OK) {
        ESP_LOGE(TAG, "ISR服务安装失败");
        return ESP_FAIL;
    }

    // 4.2 绑定中断处理函数（this指针作为参数传递给ISR）
    if (gpio_isr_handler_add(rx_pin_, rx_isr_handler, this) != ESP_OK) {
        ESP_LOGE(TAG, "中断处理函数注册失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "红外编码器初始化成功");
    return ESP_OK;
}

// 中断处理函数（必须用IRAM_ATTR修饰，放入IRAM避免缓存问题）
void IRAM_ATTR IREncoder::rx_isr_handler(void* arg) {
    // 将void*转换为IREncoder实例指针
    IREncoder* encoder = static_cast<IREncoder*>(arg);
    if (encoder) {
        encoder->handle_pulse();  // 调用成员函数处理脉冲
    }
}

// 脉冲处理（中断中调用，需轻量）
void IREncoder::handle_pulse() {
    pulse_count_++;  // 计数+1
    // 释放信号量（通知任务有新脉冲，可选操作）
    xSemaphoreGiveFromISR(pulse_sem_, NULL);
}

// 获取当前脉冲数（任务中调用）
int IREncoder::get_pulse_count() {
    return pulse_count_;  // 简单返回计数（实际可加锁，测试简化）
}

// 重置脉冲计数
void IREncoder::reset_pulse_count() {
    pulse_count_ = 0;
    ESP_LOGI(TAG, "脉冲计数已重置");
}

// 计算距离（脉冲数/每米脉冲数）
float IREncoder::get_distance() {
    return pulse_count_ / pulse_per_meter_;
}

// 重新设置车轮直径（用于校准）
void IREncoder::set_wheel_diameter(float diameter) {
    wheel_diameter_ = diameter;
    pulse_per_meter_ = 20.0f / (M_PI * wheel_diameter_);
    ESP_LOGI(TAG, "车轮直径更新为: %.2f米，每米脉冲数: %.1f", diameter, pulse_per_meter_);
}
```


### 四、编写主程序（app_main.cpp）
实现测试逻辑，验证红外计数功能：
```cpp
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ir_encoder.h"
#include "esp_log.h"

// 日志标签
static const char* TAG = "MAIN";

// 硬件引脚定义（根据你的开发板修改！）
#define IR_TX_PIN GPIO_NUM_18  // 红外发射引脚
#define IR_RX_PIN GPIO_NUM_17  // 红外接收引脚

// 测试任务：周期性打印脉冲数和距离
void test_task(void* pvParameters) {
    IREncoder* encoder = static_cast<IREncoder*>(pvParameters);
    
    while (1) {
        // 获取当前状态
        int pulse = encoder->get_pulse_count();
        float distance = encoder->get_distance();
        
        // 打印日志（ESP-IDF标准日志函数，支持不同级别）
        ESP_LOGI(TAG, "脉冲数: %d, 距离: %.2f米", pulse, distance);
        
        // 每500ms刷新一次
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// 主函数（ESP-IDF程序入口）
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "红外计数模块测试程序启动");

    // 1. 创建红外编码器实例（车轮直径0.06米=6厘米）
    IREncoder encoder(0.06f, IR_TX_PIN, IR_RX_PIN);
    
    // 2. 初始化硬件
    esp_err_t ret = encoder.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化失败，程序退出");
        return;
    }

    // 3. 创建测试任务（FreeRTOS任务，处理延时和打印）
    xTaskCreate(
        test_task,        // 任务函数
        "test_task",      // 任务名称
        4096,             // 栈大小（字节）
        &encoder,         // 传递给任务的参数
        5,                // 任务优先级
        NULL              // 任务句柄（不保存）
    );

    // 4. 主任务可以继续做其他事，这里简单循环
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```


### 五、配置CMakeLists.txt
1. 项目根目录`CMakeLists.txt`（保持默认，无需修改）：
```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ir_encoder_test)
```

2. `main/CMakeLists.txt`（指定编译源文件）：
```cmake
idf_component_register(
    SRCS "app_main.cpp" "ir_encoder.cpp"  # 源文件列表
    INCLUDE_DIRS ""                       # 头文件目录
    REQUIRES driver freertos              # 依赖的组件（driver包含GPIO/LEDC，freertos是系统库）
)
```


### 六、硬件连接与测试步骤
#### 1. 硬件准备
- 红外发射管（接IR_TX_PIN，如GPIO18）
- 红外接收管（接IR_RX_PIN，如GPIO17）
- 编码盘（安装在车轮上，随车轮转动遮挡红外信号）
- 电源（确保红外发射管有足够电流，可加限流电阻）

#### 2. 编译与下载
```bash
idf.py set-target esp32s3  # 根据你的开发板型号设置（如esp32、esp32c3等）
idf.py menuconfig          # 如需修改配置（如时钟、调试级别），一般默认即可
idf.py build               # 编译
idf.py -p COMx flash monitor  # 下载并查看日志（COMx替换为你的端口）
```

#### 3. 功能测试
- **步骤1：验证初始化**  
  观察日志是否输出`红外编码器初始化成功`，若失败检查引脚是否被占用（可通过`menuconfig`查看其他模块引脚）。

- **步骤2：测试脉冲计数**  
  手动转动车轮，观察日志中`脉冲数`是否随转动增加，若不增加：  
  - 检查红外发射/接收管是否对准编码盘  
  - 用手机摄像头观察红外发射管是否发光（手机可看到红外光）  
  - 确认中断引脚配置正确（是否为双边沿触发）

- **步骤3：校准距离计算**  
  推动小车移动1米，观察日志中`距离`是否接近1米，若误差大：  
  - 调用`set_wheel_diameter`调整车轮直径（实际测量车轮直径输入）  
  - 检查编码盘每圈脉冲数是否为20（若不同，修改`pulse_per_meter_`计算逻辑）


### 七、可直接复用的ESP-IDF功能
1. **日志系统**：`ESP_LOGI`（信息）、`ESP_LOGE`（错误）、`ESP_LOGD`（调试）等，自动包含时间戳和标签，无需自己实现。
2. **GPIO控制**：`gpio_config`、`gpio_set_level`等函数，直接操作引脚，无需寄存器级编程。
3. **PWM生成**：`ledc_timer_config`、`ledc_channel_config`，硬件生成稳定高频信号（38kHz红外常用），比软件延时可靠。
4. **中断处理**：`gpio_install_isr_service`、`gpio_isr_handler_add`，简化中断注册流程，支持C++类成员函数间接调用（通过静态函数+this指针）。
5. **FreeRTOS任务**：`xTaskCreate`、`vTaskDelay`，方便实现多任务（如计数中断+数据处理+日志打印分离）。


### 八、问题排查与学习要点
1. **中断不触发**：  
   - 检查引脚号是否正确（注意开发板丝印与GPIO编号可能不同）  
   - 确保`gpio_config`中`intr_type`设置正确（双边沿/上升沿/下降沿）  
   - 用示波器测量接收引脚是否有信号跳变

2. **脉冲计数不准**：  
   - 增加硬件滤波（在接收管引脚加100nF电容抗干扰）  
   - 在`handle_pulse`中添加简单防抖（如检测连续2次跳变才计数）

3. **学习重点**：  
   - 掌握“类+中断”的结合方式（静态函数转发）  
   - 理解`volatile`关键字在中断变量中的作用（防止编译器优化导致数据不一致）  
   - 学会用信号量（`SemaphoreHandle_t`）实现中断与任务的同步（可选扩展：在`test_task`中用`xSemaphoreTake`等待脉冲，减少轮询）


通过这个流程，你可以从硬件连接、代码编写、调试到功能验证，完整掌握一个模块的开发测试过程。后续整合到现有项目时，只需将`ir_encoder.h`和`ir_encoder.cpp`复制到`main/motor`目录，在`MotorControl`类中添加实例化和调用即可。