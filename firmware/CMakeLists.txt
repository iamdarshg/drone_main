cmake_minimum_required(VERSION 3.13)

# Set project name and languages
project(drone_firmware C ASM)

# Set standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler flags for both native and cross-compilation
if(CMAKE_CROSSCOMPILING)
	# Cross-compilation flags for LPC4330
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror=return-type")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DLPC43XX -DCORE_M4")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DFREE_RTOS")
    
	# Define executable target for embedded
	set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
	set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
else()
	# Native compilation flags for host testing
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHOST_BUILD")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DDEBUG_BUILD")
endif()

# Include directories that all modules need
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/Core
	${CMAKE_CURRENT_SOURCE_DIR}/Drivers
	${CMAKE_CURRENT_SOURCE_DIR}/Middleware
	${CMAKE_CURRENT_SOURCE_DIR}/App
	${CMAKE_CURRENT_SOURCE_DIR}/Config
	${CMAKE_CURRENT_SOURCE_DIR}/Utils
	${CMAKE_CURRENT_SOURCE_DIR}/M0
)

# Add source directories
add_subdirectory(Core)
add_subdirectory(Drivers)
add_subdirectory(Middleware)
add_subdirectory(App)
add_subdirectory(Config)
add_subdirectory(Utils)

# Only add M0 for cross-compilation
if(CMAKE_CROSSCOMPILING)
	add_subdirectory(M0)
endif()

# Create main executable
if(CMAKE_CROSSCOMPILING)
	# For LPC4330: create .elf executable
	add_executable(drone_firmware.elf)
    
	# Link all libraries
	target_link_libraries(drone_firmware.elf PRIVATE
		Core
		Drivers
		Middleware
		App
		Config
		Utils
		M0
	)
    
	# Add linker script if it exists
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld)
		set_target_properties(drone_firmware.elf PROPERTIES
			LINK_FLAGS "-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
		)
	endif()
    
	# Generate additional output formats
	add_custom_command(TARGET drone_firmware.elf POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:drone_firmware.elf> ${CMAKE_BINARY_DIR}/drone_firmware.hex
		COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:drone_firmware.elf> ${CMAKE_BINARY_DIR}/drone_firmware.bin
		COMMAND ${CMAKE_SIZE} $<TARGET_FILE:drone_firmware.elf>
		COMMENT "Generating hex and bin files, showing size"
	)
else()
	# For native: create test executable
	add_executable(drone_firmware_test)
    
	# Link all libraries except M0 (not needed for host)
	target_link_libraries(drone_firmware_test PRIVATE
		Core
		Drivers
		Middleware
		App
		Config
		Utils
		m  # Math library
	)
endif()

# Print build configuration
message(STATUS "Building drone firmware:")
message(STATUS "  Cross-compiling: ${CMAKE_CROSSCOMPILING}")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  C Flags: ${CMAKE_C_FLAGS}")
if(CMAKE_CROSSCOMPILING)
	message(STATUS "  Target: LPC4330 (ARM Cortex-M4)")
else()
	message(STATUS "  Target: Host (${CMAKE_SYSTEM_NAME})")
endif()

# Build instructions
message(STATUS "")
message(STATUS "Build Instructions:")
if(CMAKE_CROSSCOMPILING)
	message(STATUS "  For LPC4330: cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain_lpc4330.cmake")
	message(STATUS "  Then: make -C build")
	message(STATUS "  Output: build/drone_firmware.elf, .hex, .bin")
else()
	message(STATUS "  For host testing: cmake -S . -B build")
	message(STATUS "  Then: make -C build")
	message(STATUS "  Output: build/drone_firmware_test")
endif()
message(STATUS "")

