# Toolchain file for the GNU Arm Embedded toolchain (arm-none-eabi-gcc).
# The toolchain bin directory is found on PATH or in the default install
# folders at the first configure, or pass -DARM_TOOLCHAIN_DIR=<dir>/bin.
# All the tools are then called by their full path, so builds do not depend
# on the PATH of the shell (or IDE) that runs them.

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(ARM_TOOLCHAIN_DIR "" CACHE PATH "Directory containing arm-none-eabi-gcc (leave empty to search for it)")
if(NOT ARM_TOOLCHAIN_DIR)
  file(GLOB _default_dirs
       "C:/Program Files*/Arm GNU Toolchain arm-none-eabi/*/bin"
       "/opt/arm-gnu-toolchain-*/bin"
       "/Applications/ArmGNUToolchain/*/arm-none-eabi/bin")
  list(SORT _default_dirs ORDER DESCENDING)   # newest version first
  find_program(ARM_GCC_PATH arm-none-eabi-gcc PATHS ${_default_dirs})
  if(ARM_GCC_PATH)
    get_filename_component(_dir "${ARM_GCC_PATH}" DIRECTORY)
    set(ARM_TOOLCHAIN_DIR "${_dir}" CACHE PATH "Directory containing arm-none-eabi-gcc" FORCE)
  endif()
endif()
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
# FORCE: a build directory configured before the full path was known kept the bare names
set(CMAKE_OBJCOPY      "${_prefix}objcopy${_ext}" CACHE FILEPATH "" FORCE)
set(CMAKE_SIZE         "${_prefix}size${_ext}"    CACHE FILEPATH "" FORCE)

# Cross-compiling: don't try to link a host executable during compiler checks
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
