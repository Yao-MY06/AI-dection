# AI 宸ヤ笟璐ㄦ缁堢 - main 缁勪欢
# 灏嗗師鏈?monolithic main.cpp 鎷嗗垎涓哄涓亴璐ｆ竻鏅扮殑妯″潡

idf_component_register(
    SRCS
        "main.cpp"
        "app_video.c"
        "ui_manager.cpp"
        "ai_vision.cpp"
        "ai_vision_monitor.cpp"
        "ai_yolo_output_layout.cpp"
        "audio_capture.cpp"
        "audio_analysis.cpp"
        "audio_ai.cpp"
        "striker.cpp"
        "hardware_control.cpp"
        "sd_storage.cpp"
        "video_pipeline.cpp"
        "lighting.cpp"
        "servo.cpp"
        "servo_math.cpp"
        "ui_commands.cpp"
        # "sample_roi.cpp"  # Object Detection 鎺ョ鏍峰搧妗嗭紝鏃ц儗鏅樊鍒?ROI 鏆傚仠缂栬瘧
    INCLUDE_DIRS "."
    REQUIRES esp-dl driver esp-dsp esp32-camera fatfs sdmmc
)

# 灏嗚缁冨ソ鐨?AI 妯″瀷鎸?16 瀛楄妭瀵归綈宓屽叆鍒板浐浠朵腑
set(cmake_dir "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp-dl/fbs_loader/cmake")
include("${cmake_dir}/utilities.cmake")
target_add_aligned_binary_data(${COMPONENT_LIB} "../model/best_model.espdl" BINARY)

