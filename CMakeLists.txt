cmake_minimum_required(VERSION 3.6)
project(accel1)

enable_language(C ASM)

file(GLOB_RECURSE USER_SOURCES "Src/*.c")
file(GLOB_RECURSE HAL_SOURCES "Drivers/STM32L4xx_HAL_Driver/Src/*.c")
file(GLOB_RECURSE DISCO_SOURCES "Utilities/STM32L476G-Discovery/*.c" "Utilities/Components/*.c")
file(GLOB_RECURSE INIT_SOURCES "Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/*.c")
file(GLOB_RECURSE INIT_ASM "Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l476xx.s")
file(GLOB_RECURSE PERSO "perso/Src*.c")

include_directories(
        Inc
        Drivers/STM32L4xx_HAL_Driver/Inc
        Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Include
        Drivers/CMSIS/Device/ST/STM32L4xx/Include
        Utilities/STM32L476G-Discovery
        Utilities/Components
        perso/Inc
)


add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${HAL_SOURCES} ${INIT_SOURCES} ${LINKER_SCRIPT} ${INIT_ASM} ${DISCO_SOURCES} ${PERSO})

#target_link_libraries(${PROJECT_NAME}.elf CMSIS)
target_link_libraries(${PROJECT_NAME}.elf)

set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND arm-none-eabi-objcopy -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND arm-none-eabi-objcopy -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")


SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(OBJCOPY_COMMAND arm-none-eabi-objcopy)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32L476VGTx_FLASH.ld)


SET(CMAKE_C_FLAGS "-DHAL_LCD_MODULE_ENABLED -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -std=gnu11 -D__weak=\"__attribute__((weak))\" -D__packed=\"__attribute__((__packed__))\" -DUSE_HAL_DRIVER -DSTM32L476xx -Os -ffunction-sections -fdata-sections -g -Wall -Wno-unknown-pragmas")

SET(CMAKE_ASM_FLAGS  "-mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -g -Wa,--no-warn -x assembler-with-cpp")

SET(CMAKE_EXE_LINKER_FLAGS "-mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -T${LINKER_SCRIPT} -Wl,--start-group -lc -lm -Wl,--end-group -specs=nosys.specs -static -Wl,-cref,-u,Reset_Handler \"-Wl,-Map=${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.map\" -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80")