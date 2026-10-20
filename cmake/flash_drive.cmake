# Copies BIN onto the ST-LINK mass-storage drive (label DIS_F769NI).
# Usage: cmake -DBIN=<file.bin> [-DFLASH_DRIVE=<mount point>] -P flash_drive.cmake

if(NOT FLASH_DRIVE)
  if(CMAKE_HOST_WIN32)
    # Look for a drive that carries the ST-LINK's MBED.HTM / DETAILS.TXT
    foreach(_letter D E F G H I J K L M N O P Q R S T U V W X Y Z)
      if(EXISTS "${_letter}:/DETAILS.TXT" AND EXISTS "${_letter}:/MBED.HTM")
        set(FLASH_DRIVE "${_letter}:/")
        break()
      endif()
    endforeach()
  else()
    file(GLOB _mounts "/media/*/DIS_F769NI" "/run/media/*/DIS_F769NI" "/Volumes/DIS_F769NI")
    if(_mounts)
      list(GET _mounts 0 FLASH_DRIVE)
    endif()
  endif()
endif()

if(NOT FLASH_DRIVE OR NOT EXISTS "${FLASH_DRIVE}")
  message(FATAL_ERROR "ST-LINK drive (DIS_F769NI) not found. Plug in the board or set -DFLASH_DRIVE=<path>.")
endif()

message(STATUS "Copying ${BIN} -> ${FLASH_DRIVE}")
file(COPY "${BIN}" DESTINATION "${FLASH_DRIVE}")
message(STATUS "Done. The ST-LINK LED blinks while programming; the board resets when finished.")
