add_executable(${ProjectName}
    main.cpp max30105Utils.cpp heartRate.cpp
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    hardware_pwm 
    hardware_i2c
    hardware_adc
    pico_audio_i2s 
    FreeRTOS-Kernel-Heap4 
    )

pico_add_extra_outputs(${ProjectName})

# Pull in Raspberry Pi Pico SD
include(pico_sdk_import.cmake)

target_compile_definitions(${ProjectName} PRIVATE
        # compile time configuration of I2S
        PICO_AUDIO_I2S_MONO_INPUT=1
        #define for our example code
        USE_AUDIO_I2S=1
        
        PICO_AUDIO_I2S_DATA_PIN=13
        PICO_AUDIO_I2S_CLOCK_PIN_BASE=14
#            PICO_DEFAULT_UART=0
#            PICO_DEFAULT_UART_TX_PIN=28
#            PICO_DEFAULT_UART_RX_PIN=29
        )        

pico_add_extra_outputs(${ProjectName})
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)