add_library(i2s
    audioringbuffer.c
    i2s.c
)

target_include_directories(i2s PUBLIC
    .
)

pico_generate_pio_header(i2s ${CMAKE_CURRENT_LIST_DIR}/pio_i2s.pio)

target_link_libraries(
    i2s
    hardware_pio
    hardware_dma
    hardware_irq
    pico_multicore
    pico_stdlib
)

