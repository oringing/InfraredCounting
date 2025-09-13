# ESP-IDF 红外编码计数模块开发指南（完整代码与教程）


## 一、新建 ESP-IDF 项目
1. 打开 ESP-IDF Command Prompt，执行以下命令创建并进入项目目录：
```bash
idf.py create-project ir_encoder_test
cd ir_encoder_test
```

2. **项目核心文件结构**（重点关注以下文件，其他自动生成文件可忽略）：
```
ir_encoder_test/
├── CMakeLists.txt       # 项目全局配置（默认无需修改）
├── sdkconfig            # 编译配置（后续编译时自动生成）
└── main/                # 核心代码目录
    ├── CMakeLists.txt   # 源文件编译配置
    ├── main.cc          # 主程序入口（测试逻辑）
    ├── ir_encoder.h     # 红外编码器类头文件（声明）
    └── ir_encoder.cc    # 红外编码器类源文件（实现）
```


## 二、编写头文件（ir_encoder.h）
在 `main` 文件夹下新建 `ir_encoder.h`，定义红外编码器类（集中管理硬件配置、中断、任务逻辑）：

```cpp
#ifndef IR_ENCODER_H_
#define IR_ENCODER_H_

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// -------------------------- 硬件配置宏定义（集中管理，便于修改）--------------------------
#define IR_TX_DATA           GPIO_NUM_18       // 红外发射模块数据引脚
#define IR_RX_OUT            GPIO_NUM_17       // 红外接收模块输出引脚
#define BOOT_PIN             GPIO_NUM_0        // BOOT按键引脚（用于控制发射启停）
#define DEFAULT_IR_FREQ      1                 // 初始红外发射频率（Hz，每秒切换一次高低电平）
#define IR_TX_CHANNEL        LEDC_CHANNEL_5    // 红外发射PWM通道（可根据资源调整）
#define IR_TX_TIMER          LEDC_TIMER_3      // 红外发射PWM定时器（可根据资源调整）
#define WHEEL_DIAMETER_DEFAULT 0.066           // 默认车轮直径（米，即66mm）
#define ENCODER_SLOTS        20                // 编码盘镂空槽数量（每转一圈产生20个脉冲）
#define PI                   3.14159f          // π常量（用于距离计算）

class IREncoder {
private:
    // -------------------------- 硬件参数（通过宏定义初始化，支持构造函数重载）--------------------------
    gpio_num_t tx_pin_;           // 红外发射引脚
    gpio_num_t rx_pin_;           // 红外接收引脚
    uint32_t ir_freq_;            // 红外发射频率（Hz）
    ledc_channel_t tx_channel_;   // PWM发射通道
    ledc_timer_t tx_timer_;       // PWM发射定时器

    // -------------------------- 计数与控制参数 --------------------------
    float wheel_diameter_;        // 车轮直径（米）
    float pulse_per_meter_;       // 每米对应脉冲数（预计算，优化距离计算效率）
    volatile int pulse_count_;    // 脉冲计数（volatile防止编译器优化，中断中修改）
    SemaphoreHandle_t pulse_sem_; // 脉冲同步信号量（中断与任务通信）
    volatile bool is_transmitting_; // 发射状态：true=开启，false=关闭
    volatile bool has_received_;    // 接收标志（用于日志输出触发）
    SemaphoreHandle_t boot_sem_;    // BOOT按键信号量（中断与任务同步）
    TickType_t last_receive_time_;  // 上次接收脉冲时间（用于超时判断）

    // -------------------------- 静态中断处理函数（C++类中断必须为静态函数）--------------------------
    static void IRAM_ATTR rx_isr_handler(void* arg);  // 红外接收中断处理
    static void IRAM_ATTR boot_isr_handler(void* arg); // BOOT按键中断处理

    // -------------------------- 任务函数（静态函数，通过this指针访问实例）--------------------------
    static void transmit_task(void* arg);  // 红外发射控制任务

public:
    // -------------------------- 构造函数（支持默认参数，兼容宏定义配置）--------------------------
    IREncoder(float wheel_diameter = WHEEL_DIAMETER_DEFAULT, 
              gpio_num_t tx_pin = IR_TX_DATA, 
              gpio_num_t rx_pin = IR_RX_OUT, 
              uint32_t ir_freq = DEFAULT_IR_FREQ);
    
    // -------------------------- 核心初始化函数 --------------------------
    esp_err_t init();  // 硬件初始化（GPIO、中断、任务创建）

    // -------------------------- 脉冲与距离操作接口 --------------------------
    int get_pulse_count();        // 获取当前脉冲计数
    void reset_pulse_count();     // 重置脉冲计数
    float get_distance();         // 计算并返回行驶距离（米）
    void set_wheel_diameter(float diameter); // 动态修改车轮直径（用于校准）

    // -------------------------- 发射状态控制接口 --------------------------
    void toggle_transmit();       // 切换发射状态（开启/关闭）
    bool is_transmitting();       // 获取当前发射状态

    // -------------------------- 中断回调处理（非静态，由静态中断函数调用）--------------------------
    void handle_pulse();          // 处理接收脉冲（计数+状态更新）
    void handle_boot_press();     // 处理BOOT键按下（触发发射状态切换）

    // -------------------------- 接收状态查询接口（用于日志）--------------------------
    bool has_received();          // 检查是否收到新脉冲
    void reset_received_flag();   // 重置接收标志

    // -------------------------- 信号量访问接口（任务间同步）--------------------------
    SemaphoreHandle_t get_boot_semaphore();  // 获取BOOT按键信号量句柄

    // -------------------------- 接收时间查询接口 --------------------------
    TickType_t get_last_receive_time();      // 获取上次接收脉冲时间
};

#endif  // IR_ENCODER_H_
```


## 三、编写源文件（ir_encoder.cc）
在 `main` 文件夹下新建 `ir_encoder.cc`，实现头文件中声明的类方法：

```cpp
#include "ir_encoder.h"
#include "esp_log.h"
#include "driver/gpio.h"

// 日志标签（用于区分模块日志）
static const char* TAG = "IR_ENCODER";

// -------------------------- 构造函数实现（初始化成员变量与信号量）--------------------------
IREncoder::IREncoder(float wheel_diameter, gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t ir_freq)
    : tx_pin_(tx_pin), rx_pin_(rx_pin), ir_freq_(ir_freq),
      tx_channel_(IR_TX_CHANNEL), tx_timer_(IR_TX_TIMER),
      wheel_diameter_(wheel_diameter), pulse_count_(0), 
      is_transmitting_(false), has_received_(false), last_receive_time_(0) {
    // 创建二进制信号量（用于中断与任务同步，初始为未触发状态）
    pulse_sem_ = xSemaphoreCreateBinary();
    boot_sem_ = xSemaphoreCreateBinary();
    // 预计算每米对应脉冲数（优化距离计算效率：避免每次get_distance()重复计算）
    pulse_per_meter_ = ENCODER_SLOTS / (PI * wheel_diameter_);
}

// -------------------------- 硬件初始化函数（GPIO、中断、任务创建）--------------------------
esp_err_t IREncoder::init() {
    // 1. 配置红外发射引脚（输出模式）
    gpio_config_t tx_conf = {
        .pin_bit_mask = (1ULL << tx_pin_),  // 选中发射引脚
        .mode = GPIO_MODE_OUTPUT,           // 输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,  // 禁用上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉
        .intr_type = GPIO_INTR_DISABLE      // 禁用中断
    };
    gpio_config(&tx_conf);
    gpio_set_level(tx_pin_, 0);  // 初始低电平（关闭发射）

    // 2. 配置红外接收引脚（输入模式+上升沿中断）
    gpio_config_t rx_conf = {
        .pin_bit_mask = (1ULL << rx_pin_),  // 选中接收引脚
        .mode = GPIO_MODE_INPUT,            // 输入模式
        .pull_up_en = GPIO_PULLUP_ENABLE,   // 启用上拉（防止悬空误触发）
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉
        .intr_type = GPIO_INTR_POSEDGE      // 上升沿触发（接收管检测到信号时电平跳变）
    };
    gpio_config(&rx_conf);
    // 安装GPIO中断服务（参数0表示不使用中断标志位）
    gpio_install_isr_service(0);
    // 注册接收引脚中断处理函数（绑定this指针，实现实例访问）
    gpio_isr_handler_add(rx_pin_, rx_isr_handler, this);

    // 3. 配置BOOT按键引脚（输入模式+下降沿中断）
    gpio_config_t boot_conf = {
        .pin_bit_mask = (1ULL << BOOT_PIN), // 选中BOOT引脚
        .mode = GPIO_MODE_INPUT,            // 输入模式
        .pull_up_en = GPIO_PULLUP_ENABLE,   // 启用上拉（按下时为低电平）
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉
        .intr_type = GPIO_INTR_NEGEDGE      // 下降沿触发（按下时电平从高变低）
    };
    gpio_config(&boot_conf);
    // 注册BOOT引脚中断处理函数（绑定this指针）
    gpio_isr_handler_add(BOOT_PIN, boot_isr_handler, this);

    // 4. 创建红外发射控制任务（栈大小2048，优先级4，绑定this指针）
    xTaskCreate(transmit_task, "ir_transmit_task", 2048, this, 4, NULL);

    ESP_LOGI(TAG, "硬件初始化完成（发射引脚：%d，接收引脚：%d）", tx_pin_, rx_pin_);
    return ESP_OK;
}

// -------------------------- 红外发射任务（周期性切换高低电平）--------------------------
void IREncoder::transmit_task(void* arg) {
    IREncoder* encoder = static_cast<IREncoder*>(arg); // 转换为实例指针
    while (1) { // 任务死循环（FreeRTOS任务不可返回）
        if (encoder->is_transmitting_) {
            // 发射开启：低电平1秒 → 高电平1秒（周期2秒，频率1Hz）
            gpio_set_level(encoder->tx_pin_, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(encoder->tx_pin_, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // 发射关闭：保持低电平，降低CPU占用（延迟100ms）
            gpio_set_level(encoder->tx_pin_, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// -------------------------- 红外接收中断处理函数（静态）--------------------------
void IRAM_ATTR IREncoder::rx_isr_handler(void* arg) {
    IREncoder* encoder = static_cast<IREncoder*>(arg);
    encoder->handle_pulse(); // 调用非静态处理函数（避免静态函数无法访问实例成员）
}

// -------------------------- 脉冲处理函数（中断上下文安全）--------------------------
void IREncoder::handle_pulse() {
    pulse_count_++;                // 脉冲计数+1
    has_received_ = true;          // 标记收到新脉冲
    last_receive_time_ = xTaskGetTickCount(); // 更新上次接收时间
    // 释放信号量（通知任务处理，中断中需用FromISR版本）
    xSemaphoreGiveFromISR(pulse_sem_, NULL);
}

// -------------------------- BOOT按键中断处理函数（静态）--------------------------
void IRAM_ATTR IREncoder::boot_isr_handler(void* arg) {
    IREncoder* encoder = static_cast<IREncoder*>(arg);
    // 释放BOOT信号量（中断中需用FromISR版本）
    xSemaphoreGiveFromISR(encoder->boot_sem_, NULL);
}

// -------------------------- BOOT按键处理函数（切换发射状态）--------------------------
void IREncoder::handle_boot_press() {
    toggle_transmit(); // 切换发射状态
    ESP_LOGI(TAG, "红外发射状态切换：%s", is_transmitting_ ? "开启" : "关闭");
}

// -------------------------- 发射状态控制接口实现 --------------------------
void IREncoder::toggle_transmit() {
    is_transmitting_ = !is_transmitting_; // 翻转发射状态
}

bool IREncoder::is_transmitting() {
    return is_transmitting_; // 返回当前发射状态
}

// -------------------------- 接收状态查询接口实现 --------------------------
bool IREncoder::has_received() {
    return has_received_; // 返回是否收到新脉冲
}

void IREncoder::reset_received_flag() {
    has_received_ = false; // 重置接收标志
}

// -------------------------- 脉冲与距离接口实现 --------------------------
int IREncoder::get_pulse_count() {
    return pulse_count_; // 返回当前脉冲计数
}

void IREncoder::reset_pulse_count() {
    pulse_count_ = 0; // 重置脉冲计数（用于重新开始测量）
}

float IREncoder::get_distance() {
    // 距离计算公式：距离 = 脉冲数 × 车轮周长 / 编码盘槽数
    // 优化后：距离 = 脉冲数 / 每米对应脉冲数（pulse_per_meter_预计算）
    return static_cast<float>(pulse_count_) / pulse_per_meter_;
}

void IREncoder::set_wheel_diameter(float diameter) {
    wheel_diameter_ = diameter; // 更新车轮直径
    // 重新计算每米对应脉冲数（确保后续距离计算准确）
    pulse_per_meter_ = ENCODER_SLOTS / (PI * wheel_diameter_);
}

// -------------------------- 信号量访问接口实现 --------------------------
SemaphoreHandle_t IREncoder::get_boot_semaphore() {
    return boot_sem_; // 返回BOOT信号量句柄（供任务获取）
}

// -------------------------- 接收时间查询接口实现 --------------------------
TickType_t IREncoder::get_last_receive_time() {
    return last_receive_time_; // 返回上次接收时间（用于超时判断）
}
```


## 四、编写主程序（main.cc）
实现测试逻辑，包括任务创建、日志打印、按键处理：

```cpp
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ir_encoder.h"
#include "esp_log.h"

// 主程序日志标签
static const char* TAG = "MAIN";

// -------------------------- BOOT按键处理任务 --------------------------
void boot_task(void* pvParameters) {
    IREncoder* encoder = static_cast<IREncoder*>(pvParameters);
    while (1) {
        // 等待BOOT信号量（永久阻塞，直到按键按下）
        if (xSemaphoreTake(encoder->get_boot_semaphore(), portMAX_DELAY) == pdTRUE) {
            encoder->handle_boot_press(); // 处理按键（切换发射状态）
            // 简单防抖（避免按键抖动导致多次触发）
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// -------------------------- 测试任务（日志打印+距离计算）--------------------------
void test_task(void* pvParameters) {
    IREncoder* encoder = static_cast<IREncoder*>(pvParameters);
    bool log_enabled = false; // 日志使能标志（避免无脉冲时频繁打印）

    while (1) {
        // 1. 检查是否收到新脉冲
        if (encoder->has_received()) {
            int pulse = encoder->get_pulse_count();   // 获取脉冲数
            float distance = encoder->get_distance(); // 计算距离
            // 打印脉冲数与距离（保留2位小数）
            ESP_LOGI(TAG, "当前脉冲数: %d | 行驶距离: %.2f 米", pulse, distance);
            encoder->reset_received_flag(); // 重置接收标志
            log_enabled = true;             // 使能超时日志
        }

        // 2. 检查脉冲超时（5秒无新脉冲时打印提示）
        if (log_enabled && (xTaskGetTickCount() - encoder->get_last_receive_time()) > pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG, "未检测到新红外脉冲（当前脉冲数：%d）", encoder->get_pulse_count());
            log_enabled = false; // 禁用超时日志（避免重复打印）
        }

        // 降低任务占用（每100ms检查一次）
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// -------------------------- ESP-IDF主函数（程序入口）--------------------------
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "红外编码计数模块测试程序启动");

    // 1. 创建红外编码器实例（使用默认参数，可通过构造函数修改）
    IREncoder encoder;

    // 2. 初始化硬件（GPIO、中断、任务）
    esp_err_t ret = encoder.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "硬件初始化失败！程序退出");
        return;
    }

    // 3. 创建任务（测试任务优先级5，按键任务优先级5，可根据需求调整）
    xTaskCreate(test_task, "ir_test_task", 4096, &encoder, 5, NULL);
    xTaskCreate(boot_task, "ir_boot_task", 2048, &encoder, 5, NULL);

    // 4. 主循环（仅用于保持程序运行，无实际逻辑）
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```


## 五、配置 CMakeLists.txt
### 1. 项目根目录 CMakeLists.txt（默认无需修改）
```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ir_encoder_test) # 项目名称，需与创建时一致
```

### 2. main/CMakeLists.txt（指定编译源文件与依赖）
```cmake
idf_component_register(
    SRCS "main.cc" "ir_encoder.cc"  # 需编译的源文件列表（主程序+编码器实现）
    INCLUDE_DIRS "."                # 头文件目录（当前目录，即main文件夹）
    REQUIRES driver freertos        # 依赖的ESP-IDF组件（driver=GPIO/LEDC，freertos=任务/信号量）
)
```


## 六、硬件连接与测试步骤
### 1. 硬件准备清单
| 组件                | 连接说明                                  | 备注                                  |
|---------------------|-------------------------------------------|---------------------------------------|
| ESP32-S3 开发板     | -                                         | 其他ESP32系列（如ESP32、ESP32-C3）通用 |
| 红外发射管          | 阳极→限流电阻→IR_TX_DATA（GPIO18），阴极→GND | 限流电阻建议1kΩ（防止电流过大烧毁）    |
| 红外接收管（模块）  | VCC→3.3V，GND→GND，OUT→IR_RX_OUT（GPIO17） | 接收管需与发射管对齐，编码盘在中间    |
| 编码盘              | 安装在车轮上，随车轮转动遮挡红外信号      | 镂空槽数量需与`ENCODER_SLOTS`一致（20）|
| 杜邦线              | 用于连接各组件                            | 建议使用带锁扣杜邦线，避免接触不良    |

### 2. 编译与下载流程
1. **设置目标芯片**（根据开发板型号选择，如ESP32-C3则改为`esp32c3`）：
   ```bash
   idf.py set-target esp32s3
   ```

2. **配置项目**（默认配置可满足需求，如需修改日志级别/引脚可执行）：
   ```bash
   idf.py menuconfig
   ```
   - 日志级别：`Component config` → `Log output` → `Default log verbosity` → 选择`Info`
   - 引脚修改：直接修改`ir_encoder.h`中的宏定义（比menuconfig更直观）

3. **编译项目**（首次编译时间较长，需下载依赖组件）：
   ```bash
   idf.py build
   ```

4. **下载程序并查看日志**（将`COMx`替换为开发板实际端口，如`COM3`）：
   ```bash
   idf.py -p COMx flash monitor
   ```

### 3. 功能测试步骤
| 测试步骤                | 预期结果                                                                 |
|-------------------------|--------------------------------------------------------------------------|
| 1. 程序启动             | 日志输出 `红外编码计数模块测试程序启动` 和 `硬件初始化完成`              |
| 2. 按下BOOT键           | 日志输出 `红外发射状态切换：开启`（再次按下切换为关闭）                  |
| 3. 手动转动车轮         | 日志输出 `当前脉冲数: X | 行驶距离: Y.YY 米`（X随转动增加，Y同步更新） |
| 4. 停止转动车轮5秒      | 日志输出 `未检测到新红外脉冲（当前脉冲数：X）`                           |
| 5. 重置脉冲计数（代码） | 在`test_task`中调用`encoder.reset_pulse_count()`，脉冲数归零              |


## 七、ESP-IDF 核心功能复用说明
本项目充分利用 ESP-IDF 原生组件，避免重复造轮子，核心复用功能如下：

| 功能模块          | ESP-IDF 接口                          | 作用说明                                                                 |
|-------------------|---------------------------------------|--------------------------------------------------------------------------|
| 日志系统          | `ESP_LOGI`/`ESP_LOGE`/`ESP_LOGD`      | 自动添加时间戳/模块标签，支持分级输出（Info/Error/Debug），无需手动实现  |
| GPIO 控制         | `gpio_config`/`gpio_set_level`        | 封装底层寄存器操作，支持批量配置引脚模式，代码可移植性高                |
| 中断处理          | `gpio_install_isr_service`/`gpio_isr_handler_add` | 简化中断注册流程，支持C++类实例绑定（通过this指针）                      |
| FreeRTOS 任务     | `xTaskCreate`/`vTaskDelay`            | 实现多任务并发（发射任务/测试任务/按键任务），优先级可控                |
| 信号量同步        | `xSemaphoreCreateBinary`/`xSemaphoreTakeFromISR` | 解决中断与任务的同步问题（避免临界区数据竞争）                          |


## 八、问题排查与优化建议
### 1. 常见问题排查表
| 问题现象                | 可能原因                                  | 解决方案                                                                 |
|-------------------------|-------------------------------------------|--------------------------------------------------------------------------|
| 中断不触发              | 1. 引脚号错误；2. 中断类型配置错误；3. 硬件连接不良 | 1. 核对`ir_encoder.h`宏定义与实际连接；2. 接收管中断改为`GPIO_INTR_ANYEDGE`（双边沿）；3. 重新插拔杜邦线 |
| 脉冲计数不准            | 1. 编码盘遮挡不完全；2. 电磁干扰；3. 按键抖动 | 1. 调整发射管/接收管位置，确保完全遮挡；2. 在接收引脚并联100nF电容滤波；3. 在`handle_boot_press`中添加200ms延迟防抖 |
| 距离计算误差大          | 1. 车轮直径设置错误；2. 编码盘槽数不匹配 | 1. 测量实际车轮直径，修改`WHEEL_DIAMETER_DEFAULT`；2. 核对编码盘槽数与`ENCODER_SLOTS`（20） |
| 程序崩溃（重启）        | 1. 任务栈溢出；2. 中断中调用非ISR安全函数 | 1. 增大任务栈大小（如`test_task`栈从4096改为8192）；2. 中断中仅调用`IRAM_ATTR`标记的函数 |

### 2. 代码优化建议
1. **添加脉冲防抖**：在`handle_pulse`中添加连续检测（如2次中断间隔<10ms则视为抖动，不计数）：
   ```cpp
   static TickType_t last_pulse_time = 0;
   TickType_t current_time = xTaskGetTickCountFromISR();
   if (current_time - last_pulse_time > pdMS_TO_TICKS(10)) {
       pulse_count_++;
       last_pulse_time = current_time;
   }
   ```

2. **动态调整发射频率**：在类中添加`set_ir_freq`方法，支持运行时修改发射频率：
   ```cpp
   void IREncoder::set_ir_freq(uint32_t freq) {
       ir_freq_ = freq;
       // 重新计算脉冲周期（需同步修改transmit_task中的延迟）
   }
   ```

3. **添加距离校准功能**：通过串口指令修改车轮直径（如发送`set_diameter 0.068`设置为68mm）：
   ```cpp
   // 在test_task中添加串口接收逻辑，解析指令并调用set_wheel_diameter
   ```

