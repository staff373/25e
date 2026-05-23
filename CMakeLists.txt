cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME hal3_0)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    modular3.0/bsp_dcmotor/bsp_dcmotor.c
    modular3.0/bsp_encoder/bsp_encoder.c
    modular3.0/bsp_jy61p/bsp_jy61p.c
    modular3.0/bsp_sensor/bsp_sensor.c
    modular3.0/bsp_stepper/bsp_stepper.c
    modular3.0/pid_core/pid_core.c
    modular3.0/app_motion/app_motion.c
    modular3.0/app_sensor/app_sensor.c
    modular3.0/app_imu/app_imu.c
    modular3.0/app_vision/app_vision.c
    modular3.0/app_vision/app_vision_tools.c
    modular3.0/app_gimbal/app_gimbal.c
    modular3.0/app_aim/app_aim.c
    modular3.0/app_turn/app_turn.c
    modular3.0/app_track/app_track.c
    modular3.0/app_task/app_task.c
    modular3.0/app_bt/app_bt.c
    modular3.0/app_bt/app_bt_commands.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    modular3.0/bsp_dcmotor
    modular3.0/bsp_encoder
    modular3.0/bsp_jy61p
    modular3.0/bsp_sensor
    modular3.0/bsp_stepper
    modular3.0/pid_core
    modular3.0/app_motion
    modular3.0/app_sensor
    modular3.0/app_imu
    modular3.0/app_vision
    modular3.0/app_gimbal
    modular3.0/app_aim
    modular3.0/app_turn
    modular3.0/app_track
    modular3.0/app_task
    modular3.0/app_bt
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
