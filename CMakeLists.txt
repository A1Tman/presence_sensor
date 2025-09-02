# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PROJECT_NAME "presence_sensor")

# Allow target override:
# - pass -DIDF_TARGET=esp32c6 (standard IDF)
# - or pass -DCHIP=esp32c6 (convenience alias)
# - or set environment IDF_TARGET
if(NOT DEFINED IDF_TARGET)
    if(DEFINED CHIP)
        set(IDF_TARGET "${CHIP}")
    elseif(DEFINED ENV{IDF_TARGET})
        set(IDF_TARGET "$ENV{IDF_TARGET}")
    else()
        set(IDF_TARGET "esp32c3")
    endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJECT_NAME})
