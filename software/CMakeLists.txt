cmake_minimum_required(VERSION 3.0.0)
project(SJC_LIB VERSION 0.1.0 LANGUAGES C CXX)


# Project definitions
# ------------------------------------------------------------------------------------------------- #
include(CMakePrintHelpers)

set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")

message("CXX Flags: ${CMAKE_CXX_FLAGS}")
message("DEBUG CXX Flags: ${CMAKE_CXX_FLAGS_DEBUG}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
message("CXX Standard: ${CMAKE_CXX_STANDARD}")
# ------------------------------------------------------------------------------------------------- #

# 3rd party libs
# ------------------------------------------------------------------------------------------------- #
set(PROJECT_DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)

set(LIB_MODBUS_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/modbus)
set(LIB_MODBUS_SRCS_DIR ${LIB_MODBUS_BASE_DIR}/src)
link_directories(modbus ${LIB_MODBUS_SRCS_DIR})

# ------------------------------------------------------------------------------------------------- #

# Project libs
# ------------------------------------------------------------------------------------------------- #
get_filename_component(SJC_FW_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/sjc ABSOLUTE)
set(SJC_BASE_SRCS_DIR ${SJC_FW_BASE_DIR}/base/src/)
set(SJC_BASE_INCS_DIR ${SJC_FW_BASE_DIR}/base/include/)
set(SJC_TYPES_SRCS_DIR ${SJC_FW_BASE_DIR}/system_types/src/)
set(SJC_TYPES_INCS_DIR ${SJC_FW_BASE_DIR}/system_types/include/)                                                

set(SJC_INTERFACE_SRCS_DIR  ${PROJECT_DEPS_DIR}/sjc_interface/src)
set(SJC_INTERFACE_INCS_DIR  ${PROJECT_DEPS_DIR}/sjc_interface/include)

add_library(sjc_interface   ${SJC_INTERFACE_SRCS_DIR}/sjc_interface.cpp
                            ${SJC_TYPES_SRCS_DIR}/
                            ${SJC_TYPES_SRCS_DIR}/helpers.c
)
target_link_libraries(sjc_interface PUBLIC modbus)
target_include_directories(sjc_interface PUBLIC ${SJC_INTERFACE_INCS_DIR} 
                                                ${SJC_BASE_INCS_DIR} 
                                                ${SJC_TYPES_INCS_DIR} 
                                                ${LIB_MODBUS_SRCS_DIR}                                                   
)
# ------------------------------------------------------------------------------------------------- #

# Project executables
# ------------------------------------------------------------------------------------------------- #
set(PROJECT_SRCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(sjc_read_test ${PROJECT_SRCS_DIR}/sjc_read_registers.cpp )
target_link_libraries(sjc_read_test PUBLIC modbus sjc_interface)
target_include_directories(sjc_read_test PUBLIC ${LIB_MODBUS_SRCS_DIR} ${PROJECT_INCS_DIR} ${SJC_INTERFACE_INCS_DIR})

add_executable(sjc_write_test ${PROJECT_SRCS_DIR}/sjc_write_registers.cpp )
target_link_libraries(sjc_write_test PUBLIC modbus sjc_interface)
target_include_directories(sjc_write_test PUBLIC ${LIB_MODBUS_SRCS_DIR} ${PROJECT_INCS_DIR} ${SJC_INTERFACE_INCS_DIR})

add_executable(sjc_read_write_test ${PROJECT_SRCS_DIR}/sjc_read_write_registers.cpp )
target_link_libraries(sjc_read_write_test PUBLIC modbus sjc_interface)
target_include_directories(sjc_read_write_test PUBLIC ${LIB_MODBUS_SRCS_DIR} ${PROJECT_INCS_DIR} ${SJC_INTERFACE_INCS_DIR})

add_executable(sjc_read_write_test2 ${PROJECT_SRCS_DIR}/sjc_read_write_registers2.cpp )
target_link_libraries(sjc_read_write_test2 PUBLIC modbus sjc_interface)
target_include_directories(sjc_read_write_test2 PUBLIC ${LIB_MODBUS_SRCS_DIR} ${PROJECT_INCS_DIR} ${SJC_INTERFACE_INCS_DIR})

add_executable(sjc_write_control_ref ${PROJECT_SRCS_DIR}/sjc_write_control_ref.cpp )
target_link_libraries(sjc_write_control_ref PUBLIC modbus sjc_interface)
target_include_directories(sjc_write_control_ref PUBLIC ${LIB_MODBUS_SRCS_DIR} ${PROJECT_INCS_DIR} ${SJC_INTERFACE_INCS_DIR})
# ------------------------------------------------------------------------------------------------- #