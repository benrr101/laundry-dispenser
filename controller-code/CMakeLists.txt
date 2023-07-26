cmake_minimum_required(VERSION 3.0)
project(laundry_dispensor_controller_code C)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_STANDARD 99)

# Set microcontroller environment variables
set(MCU "attiny44")         # Target MCU
set(MCU_FREQ "1000000L")    # MCU frequency in Hz (available as #define)
set(PROG_ID "pickit4_isp")  # Programmer ID - for use in avrdude (see `avrdude -c ?`)
set(PROG_PART "t44")        # Target MCU part number - for use in avrdude (see `avrdude -p ?`)

# Define files for the project
set (SOURCE_LIST
        main.c
)

############################################################################
# DO NOT EDIT BELOW THIS LINE

# Set compiler/linker options
add_compile_options(-mmcu=${MCU})
add_compile_definitions(F_CPU=${MCU_FREQ})

add_link_options(-mmcu=${MCU} -Wl,-Map=${PROJECT_NAME}.map)

# Define the binary to build
add_executable(${PROJECT_NAME}.elf ${SOURCE_LIST})

# Post build step to generate Intel hex format file
add_custom_command(
        TARGET ${PROJECT_NAME}.elf
        POST_BUILD
        COMMAND ${OBJ_COPY} -j .text -j .data -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
        BYPRODUCTS ${PROJECT_NAME}.hex
)

# Burn fuses target ########################################################


# Flash target #############################################################
add_custom_target(flash
        COMMAND ${AVRDUDE} -c ${PROG_ID} -p ${PROG_PART} -U flash:w:${PROJECT_NAME}.hex:i
        DEPENDS ${PROJECT_NAME}.hex
        )