# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/Users/hemengning/pico/pico-sdk")
# 使用本地picotool，避免删除build后需要从github下载一遍
# 强制设置 PICOTOOL_EXECUTABLE 变量
set(PICOTOOL_EXECUTABLE "/Users/hemengning/pico/picotool/build/picotool" CACHE FILEPATH "Path to custom picotool" FORCE)
if(NOT TARGET picotool)
    message(STATUS "Using custom picotool: ${PICOTOOL_EXECUTABLE}")
    add_executable(picotool IMPORTED GLOBAL)
    set_property(TARGET picotool PROPERTY IMPORTED_LOCATION ${PICOTOOL_EXECUTABLE})
    add_custom_target(picotoolBuild)
else()
    message(STATUS "Overriding picotool location")
    set_property(TARGET picotool PROPERTY IMPORTED_LOCATION ${PICOTOOL_EXECUTABLE})
    get_target_property(_dep_list picotool IMPORTED_LINK_DEPENDENT_LIBRARIES)
    if(_dep_list)
        set_property(TARGET picotool PROPERTY IMPORTED_LINK_DEPENDENT_LIBRARIES "")
    endif()
endif()


set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(mcu C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(mcu
  ansi.c
  foc.c
  nr_micro_shell.c
  ansi_port.c
  main.c
  )
target_compile_options(mcu PRIVATE
    -Wall       # 启用所有常规警告
)
pico_set_program_name(mcu "mcu")
pico_set_program_version(mcu "0.1")

pico_enable_stdio_uart(mcu 0)
pico_enable_stdio_usb(mcu 1)

# Add the standard library to the build
target_link_libraries(mcu
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(mcu PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(mcu
        hardware_spi
        hardware_adc
        hardware_pwm
        hardware_dma
        hardware_flash
        pico_multicore
        )

pico_add_extra_outputs(mcu)

