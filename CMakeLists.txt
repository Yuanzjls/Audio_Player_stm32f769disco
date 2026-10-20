cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")
endif()

project(Audio_Player_stm32f769disco C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# ---------------------------------------------------------------------------
# STemWin binary library (not redistributable, so it is not in the repo).
# Get it from the STM32CubeF7 package (st.com), folder
#   Middlewares/ST/STemWin/Lib/  -> STemWin_CM7_OS_wc32_ot_ARGB.a (GCC, CM7, with OS)
# and copy it to STemWin/Lib/, or pass -DSTEMWIN_LIB=<path to .a>.
# The headers in STemWin/Include are version 5.44 (CubeF7 1.15 / 1.16).
# ---------------------------------------------------------------------------
set(STEMWIN_LIB "" CACHE FILEPATH "Path to the STemWin GCC library for Cortex-M7 with OS support")
if(NOT STEMWIN_LIB)
  file(GLOB _stemwin_candidates "${CMAKE_CURRENT_SOURCE_DIR}/STemWin/Lib/*CM7_OS*.a")
  list(SORT _stemwin_candidates)
  # Prefer the "ot" (optimized for time) variant when several are present
  foreach(_lib IN LISTS _stemwin_candidates)
    if(_lib MATCHES "_ot_")
      set(STEMWIN_LIB "${_lib}")
      break()
    endif()
  endforeach()
  if(NOT STEMWIN_LIB AND _stemwin_candidates)
    list(GET _stemwin_candidates 0 STEMWIN_LIB)
  endif()
endif()
if(STEMWIN_LIB)
  message(STATUS "STemWin library: ${STEMWIN_LIB}")
else()
  message(WARNING
    "STemWin library not found. Sources will compile but linking will fail.\n"
    "Copy STemWin_CM7_OS_wc32_ot_ARGB.a from STM32CubeF7 (Middlewares/ST/STemWin/Lib) "
    "into STemWin/Lib/ or pass -DSTEMWIN_LIB=<path>.")
endif()

# STemWin add-ons (SLIDER_SetSTSkin, ...) also ship as a binary in STM32CubeF7:
#   Middlewares/ST/STemWin/Software... or the F769I-Discovery demonstration -> STemWin_Addons_CM7_wc32.a
set(STEMWIN_ADDONS_LIB "" CACHE FILEPATH "Path to the STemWin add-ons GCC library for Cortex-M7")
if(NOT STEMWIN_ADDONS_LIB)
  file(GLOB _addons_candidates "${CMAKE_CURRENT_SOURCE_DIR}/STemWin/Lib/*Addons*CM7*.a")
  if(_addons_candidates)
    list(GET _addons_candidates 0 STEMWIN_ADDONS_LIB)
  endif()
endif()
if(STEMWIN_ADDONS_LIB)
  message(STATUS "STemWin add-ons library: ${STEMWIN_ADDONS_LIB}")
else()
  message(WARNING
    "STemWin add-ons library not found (needed for SLIDER_SetSTSkin). "
    "Copy STemWin_Addons_CM7_wc32.a into STemWin/Lib/ or pass -DSTEMWIN_ADDONS_LIB=<path>.")
endif()

# ---------------------------------------------------------------------------
# MCU flags: STM32F769NI = Cortex-M7 with double-precision FPU
# ---------------------------------------------------------------------------
set(MCU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F769NIHx_FLASH.ld")

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------
file(GLOB HAL_SOURCES "Drivers/STM32F7xx_HAL_Driver/Src/*.c")
list(FILTER HAL_SOURCES EXCLUDE REGEX "_template\\.c$")

# The BSP LCD and audio drivers are intentionally absent: the display is driven by
# STemWin/Config/LCDConf.c and main.cpp drives the WM8994 codec / SAI directly.
file(GLOB BSP_SOURCES "Drivers/BSP/STM32F769I-Discovery/*.c")

set(BSP_COMPONENT_SOURCES
  Drivers/BSP/Components/adv7533/adv7533.c
  Drivers/BSP/Components/otm8009a/otm8009a.c
  Drivers/BSP/Components/ft6x06/ft6x06.c
  Drivers/BSP/Components/wm8994/wm8994.c
)

file(GLOB CORE_SOURCES "Core/Src/*.c" "Core/Src/*.cpp")

set(FATFS_SOURCES
  FATFS/diskio.c
  FATFS/ff.c
  FATFS/ffsystem.c
  FATFS/ffunicode.c
)

set(FREERTOS_SOURCES
  FREERTOS/croutine.c
  FREERTOS/event_groups.c
  FREERTOS/heap_4.c
  FREERTOS/list.c
  FREERTOS/queue.c
  FREERTOS/stream_buffer.c
  FREERTOS/tasks.c
  FREERTOS/timers.c
  FREERTOS/portable/port.c
)

file(GLOB STEMWIN_SOURCES
  "STemWin/Config/*.c"
  "STemWin/GUI_X/*.c"
  "STemWin/STemWinTask/*.c"
)

add_executable(${PROJECT_NAME}
  startup/startup_stm32f769xx.s
  ${CORE_SOURCES}
  ${HAL_SOURCES}
  ${BSP_SOURCES}
  ${BSP_COMPONENT_SOURCES}
  ${FATFS_SOURCES}
  ${FREERTOS_SOURCES}
  ${STEMWIN_SOURCES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  USE_HAL_DRIVER
  STM32F769xx
  USE_STM32F769I_DISCO
  $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(${PROJECT_NAME} PRIVATE
  Core/Inc
  Drivers/STM32F7xx_HAL_Driver/Inc
  Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
  Drivers/CMSIS/Device/ST/STM32F7xx/Include
  Drivers/CMSIS/Include
  Drivers/BSP/STM32F769I-Discovery
  Drivers/BSP/Components/Common
  FATFS
  FREERTOS/include
  FREERTOS/portable
  STemWin/Include
  STemWin/Config
  STemWin/STemWinTask
  STemWin/STemWin_Addons
  Utilities/Fonts
)

target_compile_options(${PROJECT_NAME} PRIVATE
  ${MCU_FLAGS}
  -ffunction-sections
  -fdata-sections
  -Wall
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit>
  $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
  $<$<CONFIG:Debug>:-Og -g3>
  $<$<CONFIG:Release>:-O2 -g>
  $<$<CONFIG:MinSizeRel>:-Os -g>
)

target_link_options(${PROJECT_NAME} PRIVATE
  ${MCU_FLAGS}
  -T${LINKER_SCRIPT}
  --specs=nano.specs
  --specs=nosys.specs
  -Wl,--gc-sections
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--print-memory-usage
  -Wl,--no-warn-rwx-segments
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")

# Add-ons depend on the core library, so they come first on the link line
if(STEMWIN_ADDONS_LIB)
  target_link_libraries(${PROJECT_NAME} PRIVATE "${STEMWIN_ADDONS_LIB}")
endif()
if(STEMWIN_LIB)
  target_link_libraries(${PROJECT_NAME} PRIVATE "${STEMWIN_LIB}")
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE m)

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
