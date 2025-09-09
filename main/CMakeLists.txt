idf_component_register(
    SRCS "main.cc" "ir_encoder.cc"  # 源文件列表
    INCLUDE_DIRS ""                       # 头文件目录
    REQUIRES driver freertos              # 依赖的组件（driver包含GPIO/LEDC，freertos是系统库）
)
