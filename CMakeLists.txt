cmake_minimum_required(VERSION 3.22.1)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_drdk VERSION 1.2.4)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")

# Select the prebuilt library for this platform. DRDK is shipped as a self-contained shared library.
message(STATUS "OS: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(DRDK_LIB "libflexiv_drdk.x86_64-linux-gnu.so")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(DRDK_LIB "libflexiv_drdk.aarch64-linux-gnu.so")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
else()
    message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} is currently not supported.")
endif()

# Parse expected hash of the library file
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${DRDK_LIB}.sha256 EXPECTED_HASH)

# Download the library file from GitHub
set(DRDK_LIB_URL https://github.com/flexivrobotics/flexiv_drdk/releases/download/v${PROJECT_VERSION}/${DRDK_LIB})
message(STATUS "Downloading ${DRDK_LIB_URL} ...")
file(DOWNLOAD ${DRDK_LIB_URL} ${CMAKE_CURRENT_BINARY_DIR}/${DRDK_LIB}
     SHOW_PROGRESS
     TLS_VERIFY ON
     EXPECTED_HASH SHA256=${EXPECTED_HASH}
     STATUS status)
list(GET status 0 code)
if(code EQUAL 0)
  message(STATUS "Download finished")
else()
  message(FATAL_ERROR "Download failed: ${status}")
endif()

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================
# flexiv_rdk
find_package(flexiv_rdk REQUIRED)
message(STATUS "Found flexiv_rdk v${flexiv_rdk_VERSION}: ${flexiv_rdk_DIR}")

# ===================================================================
#      CREATE LIBRARY
# ===================================================================
# Create the library target with a dummy source. The dummy output is replaced at install time by
# the downloaded prebuilt shared library (see FlexivInstallLibrary).
add_library(${PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/dummy.cpp)

# Create an alias of the target with flexiv namespace
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to dependencies
# flexiv_rdk is the only remaining public dependency, because rdk:: types appear in the public
# flexiv::drdk API headers. Everything else is embedded in the shared library.
target_link_libraries(${PROJECT_NAME} PUBLIC
  flexiv::flexiv_rdk
)

# Use moderate compiler warning option
if(CMAKE_HOST_UNIX)
  target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)
else()
  target_compile_options(${PROJECT_NAME} PUBLIC /W1)
endif()

# Install the library
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()
