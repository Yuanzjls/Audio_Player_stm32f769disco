# Host unit tests: the board-independent parts of the firmware (ID3 tags, WAV
# and MP3 decoding, spectrum analysis) built for the PC, with fakes for the
# SD card, the log and the audio output (support/fakes.c).
#
#   cmake -S tests -B build/tests -G Ninja
#   cmake --build build/tests
#   ctest --test-dir build/tests --output-on-failure
cmake_minimum_required(VERSION 3.20)
project(audio_player_tests C)

set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)   # line numbers in failures and backtraces
endif()
set(CMAKE_C_STANDARD_REQUIRED ON)

option(SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(DSP  ${ROOT}/Middlewares/CMSIS-DSP)

if(NOT EXISTS ${ROOT}/Middlewares/minimp3/minimp3.h OR NOT EXISTS ${DSP}/Include/arm_math.h)
  message(FATAL_ERROR "Submodules missing: run 'git submodule update --init'")
endif()

# Third-party code, built as on the target (minimp3 without SIMD, CMSIS-DSP's
# FFT); __GNUC_PYTHON__ is CMSIS-DSP's switch for builds without CMSIS-Core
add_library(third_party STATIC
  ${ROOT}/App/Src/minimp3_impl.c
  ${ROOT}/FATFS/ffunicode.c
  # Only the FFT and magnitude functions the spectrum uses (the grouped
  # *Functions.c files would need dead-code removal by the linker, which
  # Windows hosts do not do for them)
  ${DSP}/Source/TransformFunctions/arm_rfft_fast_f32.c
  ${DSP}/Source/TransformFunctions/arm_rfft_fast_init_f32.c
  ${DSP}/Source/TransformFunctions/arm_cfft_f32.c
  ${DSP}/Source/TransformFunctions/arm_cfft_init_f32.c
  ${DSP}/Source/TransformFunctions/arm_cfft_radix8_f32.c
  ${DSP}/Source/TransformFunctions/arm_bitreversal2.c
  ${DSP}/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c
  ${DSP}/Source/CommonTables/arm_common_tables.c
  ${DSP}/Source/CommonTables/arm_const_structs.c
)
# On Windows hosts, a wrapper for ff.h comes first (see support/win32/ff.h)
if(WIN32)
  target_include_directories(third_party BEFORE PUBLIC support/win32)
endif()
target_include_directories(third_party
  PUBLIC  ${ROOT}/FATFS ${ROOT}/Middlewares/minimp3 ${DSP}/Include
  PRIVATE ${DSP}/PrivateInclude
)
target_compile_definitions(third_party PUBLIC MINIMP3_ONLY_MP3 MINIMP3_NO_SIMD __GNUC_PYTHON__)
target_compile_options(third_party PRIVATE -w)

# The firmware code under test, unchanged, and the fakes it runs against
add_library(app_under_test STATIC
  ${ROOT}/App/Src/id3.c
  ${ROOT}/App/Src/wav.c
  ${ROOT}/App/Src/decoder.c
  ${ROOT}/App/Src/decoder_wav.c
  ${ROOT}/App/Src/decoder_mp3.c
  ${ROOT}/App/Src/spectrum.c
  support/fakes.c
  support/check.c
)
target_include_directories(app_under_test PUBLIC
  support
  ${ROOT}/App/Inc
  ${ROOT}/App/Src
  ${ROOT}/Board/Inc
)
target_link_libraries(app_under_test PUBLIC third_party)
target_compile_options(app_under_test PRIVATE -Wall -Wextra -Wno-unused-parameter)
if(NOT WIN32)
  target_link_libraries(app_under_test PUBLIC m)
endif()

if(SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
  target_compile_options(third_party PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_compile_options(app_under_test PRIVATE -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
endif()

enable_testing()
foreach(name id3 wav mp3 spectrum)
  add_executable(test_${name} test_${name}.c)
  target_link_libraries(test_${name} PRIVATE app_under_test)
  target_compile_options(test_${name} PRIVATE -Wall -Wextra -Wno-unused-parameter)
  target_compile_definitions(test_${name} PRIVATE TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
  add_test(NAME ${name} COMMAND test_${name})
endforeach()
