

add_library(mbed-my_f405 INTERFACE)

target_include_directories(mbed-my_f405
    INTERFACE
        .
)

target_sources(mbed-my_f405
    INTERFACE
        PeripheralPins.c
)

target_link_libraries(mbed-my_f405 INTERFACE mbed-stm32f405xx)
