cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

set(CMAKE_FILES ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_TOOLCHAIN_FILE    ${CMAKE_FILES}/toolchain-gcc-arm-embedded.cmake)

project(atsame5x_example_app)

include(${CMAKE_FILES}/CPM_setup.cmake)


#-----------------------------------------------------------------------
# Build settings
#-----------------------------------------------------------------------

set(EXE_NAME                atsame5x_app)
set(FLASH_ADDR              0x00000000)
set(FLASH_CFG               atsame5x) # does not exist
set(DEBUG_BREAKPOINT_LIMIT  4)
set(DEBUG_WATCHPOINT_LIMIT  2)


# default settings
set(OPTIMIZE s)
set(BLACKMAGIC_DEV /dev/ttyBmpGdb)
set(POWER_TARGET "no")

# Include custom settings
# (if this file does not exist, copy it from config.cmake.example)
include(${CMAKE_SOURCE_DIR}/config.cmake)

message(STATUS "Config OPTIMIZE: ${OPTIMIZE}")
message(STATUS "Config BLACKMAGIC_DEV: ${BLACKMAGIC_DEV}")
message(STATUS "Config POWER_TARGET: ${POWER_TARGET}")

set(SYSTEM_LIBRARIES    m c gcc)

set(FLAGS_M4 "-mcpu=cortex-m4")

set(C_FLAGS "-O${OPTIMIZE} -g3 -c -fmessage-length=80 -fno-builtin   \
    -ffunction-sections -fdata-sections -std=gnu99 -mthumb      \
    -fdiagnostics-color=auto")
set(C_FLAGS_WARN "-Wall -Wextra -Wno-unused-parameter           \
    -Wshadow -Wpointer-arith -Winit-self -Wstrict-overflow=2")

set(L_FLAGS "-fmessage-length=80 -specs=nano.specs \
    -mthumb -Wl,--gc-sections")

# we don't use -nostdlib for this project

add_definitions("${FLAGS_M4} ${C_FLAGS} ${C_FLAGS_WARN}")

# Select Device
add_definitions(-D__SAME53J20A__)


set(ELF_PATH            "${CMAKE_CURRENT_BINARY_DIR}/${EXE_NAME}")
set(EXE_PATH            "${ELF_PATH}.bin")
set(FLASH_FILE          ${PROJECT_BINARY_DIR}/flash.cfg)

#------------------------------------------------------------------------------
# CPM Modules
#------------------------------------------------------------------------------

include_directories("config")

CPM_AddModule("chip_atsame5x"
    GIT_REPOSITORY "https://github.com/JitterCompany/chip_atsame5x.git"
    GIT_TAG "master")
    # GIT_TAG "2.1"
    # SOURCE_DIR "../atsame5x")

CPM_AddModule("mcu_debug"
   GIT_REPOSITORY "https://github.com/JitterCompany/mcu_debug.git"
   GIT_TAG "2.1")

CPM_Finish()


set(LINKER_FILES "-L .. -T same53j20a_flash.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${L_FLAGS} \
${LINKER_FILES} ${FLAGS_M4}")


#-----------------------------------------------------------------------
# Setup source
#-----------------------------------------------------------------------

include_directories("src") # todo is this necessary?

file(GLOB SOURCES
    "src/*.c"
)

set(CMAKE_SYSTEM_NAME Generic)


#-----------------------------------------------------------------------
# Setup executable
#-----------------------------------------------------------------------
add_executable(${EXE_NAME} ${SOURCES})
target_link_libraries(${EXE_NAME} ${CPM_LIBRARIES})
target_link_libraries(${EXE_NAME} ${SYSTEM_LIBRARIES})

add_custom_target(bin
    # empty flash file
    COMMAND > "${FLASH_FILE}"

    DEPENDS ${EXE_NAME}
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXE_NAME} ${EXE_NAME}.bin

    # append flash file
    COMMAND echo "${PROJECT_BINARY_DIR}/${EXE_NAME}.bin ${FLASH_ADDR} ${FLASH_CFG}" >> "${PROJECT_BINARY_DIR}/flash.cfg"
    )

add_dependencies(flash bin)
add_dependencies(debug bin)
