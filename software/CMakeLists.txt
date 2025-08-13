cmake_minimum_required(VERSION 3.10.0)
project(sjc_interface VERSION 0.1.0 LANGUAGES C CXX)

find_pacKage(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# Project definitions
# ------------------------------------------------------------------------------------------------- #
include(CMakePrintHelpers)
set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

message("-- CXX Flags: ${CMAKE_CXX_FLAGS}")
message("-- DEBUG CXX Flags: ${CMAKE_CXX_FLAGS_DEBUG}")
message("-- CXX Standard: ${CMAKE_CXX_STANDARD}")
# ------------------------------------------------------------------------------------------------- #

# 3rd party libs
# ------------------------------------------------------------------------------------------------- #
find_library(MODBUS_LIBRARY NAMES modbus REQUIRED)
if(NOT MODBUS_LIBRARY)
    message(FATAL_ERROR "Modbus library not found")
endif()
# ------------------------------------------------------------------------------------------------- #

# Project libs
# ------------------------------------------------------------------------------------------------- #
set(SJC_INTERFACE_LIB_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(SJC_INTERFACE_SRCS_DIR      ${SJC_INTERFACE_LIB_DIR}/src)
set(SJC_INTERFACE_INCS_DIR      ${SJC_INTERFACE_LIB_DIR}/include)

add_library(${PROJECT_NAME}  SHARED ${SJC_INTERFACE_SRCS_DIR}/${PROJECT_NAME}.cpp 
                                  ${SJC_INTERFACE_SRCS_DIR}/helpers.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC modbus)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${SJC_INTERFACE_INCS_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
message("-- Configured library: ${PROJECT_NAME}")

# ------------------------------------------------------------------------------------------------- #

# Project executables
# ------------------------------------------------------------------------------------------------- #
set(PROJECT_SRCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(TEST_SOURCES
    sjc_read_boards_test.cpp
    sjc_write_control_ref.cpp
)

foreach(source IN LISTS TEST_SOURCES)
    get_filename_component(exec_name ${source} NAME_WE)
    add_executable(${exec_name} ${PROJECT_SRCS_DIR}/${source})
    target_link_libraries(${exec_name} PUBLIC ${PROJECT_NAME})
    target_include_directories(${exec_name} PUBLIC ${SJC_INTERFACE_INCS_DIR})
    message("-- Configured executable: ${exec_name}")
endforeach()
# ------------------------------------------------------------------------------------------------- #

# Library installation
# ------------------------------------------------------------------------------------------------- #
include(GNUInstallDirs)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds are not allowed. Please use a separate build directory.")
endif()

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY ${SJC_INTERFACE_INCS_DIR}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# Exportar a configuração da biblioteca
install(EXPORT ${PROJECT_NAME}_targets
    FILE ${PROJECT_NAME}Config.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Criar um arquivo de versão da biblioteca
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)