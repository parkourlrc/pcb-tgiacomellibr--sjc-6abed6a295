cmake_minimum_required(VERSION 3.10.0)
project(sjc_interface VERSION 0.1.0 LANGUAGES C CXX)

# ================================================================================================ #
# Project definitions
# ================================================================================================ #
# Print helpers and compilation flags
include(CMakePrintHelpers)

# Debug flags for C and C++
set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")

# Enforce C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Print configuration for debugging purposes
message("-- CXX Flags: ${CMAKE_CXX_FLAGS}")
message("-- DEBUG CXX Flags: ${CMAKE_CXX_FLAGS_DEBUG}")
message("-- CXX Standard: ${CMAKE_CXX_STANDARD}")

# ================================================================================================ #
# Dependencies: libmodbus
# ================================================================================================ #
# Try to locate libmodbus using pkg-config. If not found, fall back to find_library.
find_package(PkgConfig REQUIRED)

pkg_check_modules(MODBUS libmodbus)

if (MODBUS_FOUND)
    message("-- Found libmodbus via pkg-config")
else()
    message("-- pkg-config did not find libmodbus, falling back to find_library")
    find_library(MODBUS_LIBRARY NAMES modbus REQUIRED)
    if(NOT MODBUS_LIBRARY)
        message(FATAL_ERROR "libmodbus not found")
    endif()
    set(MODBUS_LIBRARIES ${MODBUS_LIBRARY})
endif()

# ================================================================================================ #
# Project directories
# ================================================================================================ #
set(SJC_INTERFACE_LIB_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(SJC_INTERFACE_SRCS_DIR ${SJC_INTERFACE_LIB_DIR}/src)
set(SJC_INTERFACE_INCS_DIR ${SJC_INTERFACE_LIB_DIR}/include)

set(PROJECT_SRCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ================================================================================================ #
# Library target
# ================================================================================================ #
# Build the shared library "sjc_interface"
add_library(${PROJECT_NAME} SHARED
    ${SJC_INTERFACE_SRCS_DIR}/${PROJECT_NAME}.cpp
    ${SJC_INTERFACE_SRCS_DIR}/sjc_realtime_interface.cpp
    ${SJC_INTERFACE_SRCS_DIR}/helpers.cpp
)

# Include directories for this library
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${SJC_INTERFACE_INCS_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../firmware/sjc/common/include>
    ${MODBUS_INCLUDE_DIRS}
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Link against libmodbus
target_link_libraries(${PROJECT_NAME} PUBLIC ${MODBUS_LIBRARIES})

message("-- Configured library: ${PROJECT_NAME}")

# ================================================================================================ #
# Executables (tests and examples)
# ================================================================================================ #
find_package(nlohmann_json REQUIRED)
# List of example/test sources
set(TEST_SOURCES
    sjc_write_log.cpp
    sjc_read_boards_test.cpp
    sjc_write_control_freq.cpp
    sjc_write_control_mode.cpp
    sjc_write_control_ref.cpp
    sjc_write_control_mode_ref.cpp
    sjc_read_write_test_threaded.cpp
    sjc_write_control_freq_mode_ref.cpp
    sjc_monitor.cpp
)

# Each source file becomes an independent executable
foreach(source IN LISTS TEST_SOURCES)
    get_filename_component(exec_name ${source} NAME_WE)
    add_executable(${exec_name} ${PROJECT_SRCS_DIR}/${source})

    # Link test executables against our library and libmodbus
    target_link_libraries(${exec_name} PUBLIC ${PROJECT_NAME} ${MODBUS_LIBRARIES} nlohmann_json::nlohmann_json
)

    # Expose the sjc_interface include directory to test executables
    target_include_directories(${exec_name} PUBLIC ${SJC_INTERFACE_INCS_DIR})

    message("-- Configured executable: ${exec_name}")
endforeach()

# ================================================================================================ #
# Installation rules
# ================================================================================================ #
include(GNUInstallDirs)

# Forbid in-source builds (must use a build/ directory)
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds are not allowed. Please use a separate build directory.")
endif()

# Install library binary
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install library headers
install(DIRECTORY ${SJC_INTERFACE_INCS_DIR}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# Export CMake target configuration
install(EXPORT ${PROJECT_NAME}_targets
    FILE ${PROJECT_NAME}Config.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Generate version file for the library package
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
