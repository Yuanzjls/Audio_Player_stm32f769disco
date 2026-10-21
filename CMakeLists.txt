cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")
endif()

project(Audio_Player_stm32f769disco C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

foreach(_sub Middlewares/lvgl Middlewares/FreeRTOS-Kernel Middlewares/minimp3 Middlewares/CMSIS-DSP
             Drivers/STM32F7xx_HAL_Driver
             Drivers/CMSIS/Device/ST/STM32F7xx Drivers/BSP/STM32F769I-Discovery)
  file(GLOB _sub_files "${CMAKE_CURRENT_SOURCE_DIR}/${_sub}/*")
  if(NOT _sub_files)
    message(FATAL_ERROR "${_sub} is empty. Run: git submodule update --init")
  endif()
endforeach()

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F769NIHx_FLASH.ld")

# ---------------------------------------------------------------------------
# Settings shared by the application and LVGL
# (STM32F769NI = Cortex-M7 with double-precision FPU)
# ---------------------------------------------------------------------------
add_library(board_config INTERFACE)

target_compile_definitions(board_config INTERFACE
  USE_HAL_DRIVER
  STM32F769xx
  USE_STM32F769I_DISCO
  LV_CONF_INCLUDE_SIMPLE
  $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(board_config INTERFACE
  Core/Inc
  Drivers/STM32F7xx_HAL_Driver/Inc
  Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
  Drivers/CMSIS/Device/ST/STM32F7xx/Include
  Drivers/CMSIS/Include
  Drivers/BSP/STM32F769I-Discovery
  Drivers/BSP/Components
  Drivers/BSP/Components/Common
  FATFS
  Middlewares/FreeRTOS-Kernel/include
  Middlewares/FreeRTOS-Kernel/portable/GCC/ARM_CM4F
  Middlewares/lvgl
)

target_compile_options(board_config INTERFACE
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
  -ffunction-sections
  -fdata-sections
  $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
  $<$<CONFIG:Debug>:-Og -g3>
  $<$<CONFIG:Release>:-O2 -g>
  $<$<CONFIG:MinSizeRel>:-Os -g>
)

target_link_options(board_config INTERFACE
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
)

# ---------------------------------------------------------------------------
# LVGL (git submodule, configured by Core/Inc/lv_conf.h)
# ---------------------------------------------------------------------------
file(GLOB_RECURSE LVGL_SOURCES CONFIGURE_DEPENDS "Middlewares/lvgl/src/*.c")
add_library(lvgl STATIC ${LVGL_SOURCES})
target_link_libraries(lvgl PRIVATE board_config)
# Speed matters more than debuggability inside LVGL, even in Debug builds
target_compile_options(lvgl PRIVATE -O2 -w)

# ---------------------------------------------------------------------------
# CMSIS-DSP (submodule): only the FFT and complex-math groups are needed
# (spectrum display); unused functions and tables are removed by the linker.
# ---------------------------------------------------------------------------
set(CMSIS_DSP_DIR Middlewares/CMSIS-DSP)
add_library(cmsis_dsp STATIC
  ${CMSIS_DSP_DIR}/Source/TransformFunctions/TransformFunctions.c
  ${CMSIS_DSP_DIR}/Source/CommonTables/CommonTables.c
  ${CMSIS_DSP_DIR}/Source/ComplexMathFunctions/ComplexMathFunctions.c
)
target_include_directories(cmsis_dsp
  PUBLIC  ${CMSIS_DSP_DIR}/Include
  PRIVATE ${CMSIS_DSP_DIR}/PrivateInclude
)
target_link_libraries(cmsis_dsp PRIVATE board_config)
target_compile_options(cmsis_dsp PRIVATE -O2 -w)

# ---------------------------------------------------------------------------
# Firmware
# ---------------------------------------------------------------------------
file(GLOB HAL_SOURCES "Drivers/STM32F7xx_HAL_Driver/Src/*.c")
list(FILTER HAL_SOURCES EXCLUDE REGEX "_template\.c$")
# Patched copy with a shorter SD command timeout (see the header of the copy)
list(FILTER HAL_SOURCES EXCLUDE REGEX "stm32f7xx_ll_sdmmc\.c$")
list(APPEND HAL_SOURCES Drivers/HAL_Patches/stm32f7xx_ll_sdmmc.c)

# Only the BSP parts the player needs. The BSP LCD and audio drivers are not used:
# the display is driven by Board/Src/display.c and the codec by Board/Src/audio_out.c.
set(BSP_SOURCES
  Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.c
  Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sd.c
  Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_sdram.c
  Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_ts.c
)

set(BSP_COMPONENT_SOURCES
  Drivers/BSP/Components/otm8009a/otm8009a.c
  Drivers/BSP/Components/ft6x06/ft6x06.c
  Drivers/BSP/Components/wm8994/wm8994.c
)

set(FATFS_SOURCES
  FATFS/diskio.c
  FATFS/ff.c
  FATFS/ffsystem.c
  FATFS/ffunicode.c
)

# FreeRTOS kernel (submodule). The Cortex-M7 r1p0 in the STM32F769 uses the CM4F port.
set(FREERTOS_DIR Middlewares/FreeRTOS-Kernel)
set(FREERTOS_SOURCES
  ${FREERTOS_DIR}/event_groups.c
  ${FREERTOS_DIR}/list.c
  ${FREERTOS_DIR}/queue.c
  ${FREERTOS_DIR}/stream_buffer.c
  ${FREERTOS_DIR}/tasks.c
  ${FREERTOS_DIR}/timers.c
  ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c
  ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
  "Core/Src/*.c"
  "Board/Src/*.c"
  "App/Src/*.c"
)

add_executable(${PROJECT_NAME}
  Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f769xx.s
  Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
  ${APP_SOURCES}
  ${HAL_SOURCES}
  ${BSP_SOURCES}
  ${BSP_COMPONENT_SOURCES}
  ${FATFS_SOURCES}
  ${FREERTOS_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  Board/Inc
  App/Inc
  Middlewares/minimp3
)

# MP3 decoder: Layer III only, no SIMD (the Cortex-M7 has no NEON)
target_compile_definitions(${PROJECT_NAME} PRIVATE MINIMP3_ONLY_MP3 MINIMP3_NO_SIMD)

# Warnings for our own code
set_source_files_properties(${APP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wno-unused-parameter")
# minimp3 implementation: third-party code, and decoding speed matters even in Debug
set_source_files_properties(App/Src/minimp3_impl.c PROPERTIES COMPILE_OPTIONS "-O3;-w")
# Runs ~30 times a second: optimised in Debug builds too
set_source_files_properties(App/Src/spectrum.c PROPERTIES COMPILE_OPTIONS "-O2;-Wall;-Wextra;-Wno-unused-parameter")

target_link_options(${PROJECT_NAME} PRIVATE
  -T${LINKER_SCRIPT}
  --specs=nano.specs
  --specs=nosys.specs
  -Wl,--gc-sections
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--print-memory-usage
  -Wl,--no-warn-rwx-segments
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")

target_link_libraries(${PROJECT_NAME} PRIVATE board_config lvgl cmsis_dsp m)

# ---------------------------------------------------------------------------
# Post-build: .hex / .bin and size report
# ---------------------------------------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
  COMMENT "Generating ${PROJECT_NAME}.hex / .bin"
)

# ---------------------------------------------------------------------------
# `cmake --build <dir> --target flash`
# Uses STM32CubeProgrammer if installed, otherwise OpenOCD.
# ---------------------------------------------------------------------------
find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI
  PATHS
    "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
    "C:/Program Files (x86)/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
    "$ENV{HOME}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin"
)
file(GLOB _openocd_hints "$ENV{LOCALAPPDATA}/Microsoft/WinGet/Packages/xpack-dev-tools.openocd-xpack*/xpack-openocd-*/bin")
find_program(OPENOCD openocd PATHS ${_openocd_hints})

if(STM32_PROGRAMMER_CLI)
  add_custom_target(flash
    COMMAND "${STM32_PROGRAMMER_CLI}" -c port=SWD mode=UR -w $<TARGET_FILE:${PROJECT_NAME}> -v -rst
    DEPENDS ${PROJECT_NAME}
    USES_TERMINAL
    COMMENT "Flashing with STM32CubeProgrammer")
elseif(OPENOCD)
  add_custom_target(flash
    COMMAND "${OPENOCD}" -f board/stm32f769i-disco.cfg
            -c "program $<TARGET_FILE:${PROJECT_NAME}> verify reset exit"
    DEPENDS ${PROJECT_NAME}
    USES_TERMINAL
    COMMENT "Flashing with OpenOCD")
else()
  add_custom_target(flash
    COMMAND ${CMAKE_COMMAND} -E echo
      "No STM32_Programmer_CLI or openocd found. Use the flash_drive target instead."
    DEPENDS ${PROJECT_NAME})
endif()

# `cmake --build <dir> --target flash_drive`
# Drag-and-drop programming: the ST-LINK/V2-1 exposes a USB drive (DIS_F769NI);
# copying a .bin onto it programs the flash at 0x08000000. Needs no driver.
set(FLASH_DRIVE "" CACHE PATH "Mount point of the DIS_F769NI ST-LINK drive, e.g. F:/")
add_custom_target(flash_drive
  COMMAND ${CMAKE_COMMAND}
          -DBIN=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
          -DFLASH_DRIVE=${FLASH_DRIVE}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/flash_drive.cmake
  DEPENDS ${PROJECT_NAME}
  USES_TERMINAL
  COMMENT "Copying ${PROJECT_NAME}.bin to the ST-LINK drive")
