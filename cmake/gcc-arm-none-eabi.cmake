# Toolchain file for the GNU Arm Embedded toolchain (arm-none-eabi-gcc).
# The toolchain bin directory must be on PATH, or pass -DARM_TOOLCHAIN_DIR=<dir>/bin.

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(ARM_TOOLCHAIN_DIR "" CACHE PATH "Directory containing arm-none-eabi-gcc (leave empty to use PATH)")
if(ARM_TOOLCHAIN_DIR)
  set(_prefix "${ARM_TOOLCHAIN_DIR}/arm-none-eabi-")
else()
  set(_prefix "arm-none-eabi-")
endif()

if(CMAKE_HOST_WIN32)
  set(_ext ".exe")
else()
  set(_ext "")
endif()

set(CMAKE_C_COMPILER   "${_prefix}gcc${_ext}")
set(CMAKE_CXX_COMPILER "${_prefix}g++${_ext}")
set(CMAKE_ASM_COMPILER "${_prefix}gcc${_ext}")
set(CMAKE_OBJCOPY      "${_prefix}objcopy${_ext}" CACHE FILEPATH "")
set(CMAKE_SIZE         "${_prefix}size${_ext}"    CACHE FILEPATH "")

# Cross-compiling: don't try to link a host executable during compiler checks
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
