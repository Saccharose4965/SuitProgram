idf_component_register(
    SRCS "shell.c" "menu.c" "status.c"
    PRIV_INCLUDE_DIRS "."
    REQUIRES arm_pose gyro_calibration paint_app synth synth_service
             message_app messages settings settings_app music
             live_sync led_control led_source
             remote_control remote_control_app
             audio_policy state input input_audio_policy oled hw ui_shell
             performance_monitor flappy twenty48 tetris audio audio_i2s
             audio_player fft fft_view keyboard storage_sd led_modes led_layout
             led_layout_app power link audio_rx file_rx bt_audio bluetooth snake
             pong pong_app threedee fluid orientation paint bad_apple calculator
             microphone telemetry show_package show_runtime show_player call call_app
)
