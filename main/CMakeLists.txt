# F:/stm/p4_defect_detect/main/CMakeLists.txt

# 1. 注册组件：
# 如果你把 main.c 改成了 main.cpp，记得把这里也改了
# 同时添加 REQUIRES esp-dl 来告诉编译器链接神经网络库
idf_component_register(SRCS "main.cpp" "app_video.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp-dl driver esp-dsp esp32-camera fatfs sdmmc)

# 2. 嵌入 AI 模型：
# 假设你的 best_model.espdl 放在工程根目录的 model 文件夹里
# 这行代码会将模型转换为二进制数组，并生成 _binary_best_model_espdl_start 符号
target_add_binary_data(${COMPONENT_LIB} "../model/best_model.espdl" BINARY)