set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.28)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(hc_f460 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DHC32F460 -DUSE_DDL_DRIVER)

# Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# Uncomment for software floating point
# add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

# Define a function to process directories
function(process_directory target directories)
    foreach(dir IN LISTS directories)
        file(GLOB_RECURSE inc_files "${dir}/Inc/*")
        if(inc_files)
            target_include_directories(${target} PRIVATE "${dir}/Inc")
        endif()

        file(GLOB_RECURSE src_files "${dir}/Src/*")
        if(src_files)
            target_sources(${target} PRIVATE ${src_files})
        endif()
    endforeach()
endfunction()


# Set linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Config/linker/HC32F460xE.ld)

add_link_options(-specs=nano.specs)
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

# Create the executable target
add_executable(${PROJECT_NAME}.elf)
target_link_libraries(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/DSP/libCMSISDSP.a)
# Manually add specific files
# Include default directories
include_directories(Core/Inc Drivers/CMSIS/Core/Include Drivers/CMSIS/DSP/Include)
file(GLOB_RECURSE MANUAL_FILES "Core/Startup/*.S")

# List of directories to process
set(DIRECTORIES 
    Core
    Drivers/hc32_ll_driver
)

# Process directories to add sources and include paths
process_directory(${PROJECT_NAME}.elf "${DIRECTORIES}")
target_sources(${PROJECT_NAME}.elf PRIVATE ${MANUAL_FILES})

# Add post-build commands
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}")
