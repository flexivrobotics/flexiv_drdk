cmake_minimum_required(VERSION 3.22.0)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_drdk VERSION 1.1.0)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")

# Set static library according to platform
message(STATUS "OS: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(DRDK_LIB "libflexiv_drdk.x86_64-linux-gnu.a")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(DRDK_LIB "libflexiv_drdk.aarch64-linux-gnu.a")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
else()
    message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} is currently not supported.")
endif()

# Parse expected hash of the library file
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${DRDK_LIB}.sha256 EXPECTED_HASH)

# Download the library file from GitHub 
set(DRDK_LIB_URL https://github.com/flexivrobotics/flexiv_drdk/releases/download/v1.1/${DRDK_LIB})
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

# coal
find_package(coal REQUIRED)
message(STATUS "Found coal v${coal_VERSION}: ${coal_DIR}")

# urdfdom
find_package(urdfdom REQUIRED)
message(STATUS "Found urdfdom v${urdfdom_VERSION}: ${urdfdom_DIR}")

# assimp: dependency of coal but coal didn't handle it well so we need to find it for coal
find_package(assimp REQUIRED)
message(STATUS "Found assimp v${assimp_VERSION}: ${assimp_DIR}")

# console_bridge: dependency of urdfdom, needs to be explicitly found
find_package(console_bridge REQUIRED)
message(STATUS "Found console_bridge v${console_bridge_VERSION}: ${console_bridge_DIR}")

# tinyxml2: dependency of urdfdom, needs to be explicitly found
find_package(tinyxml2 REQUIRED)
message(STATUS "Found tinyxml2 v${tinyxml2_VERSION}: ${tinyxml2_DIR}")

# ===================================================================
#      CREATE LIBRARY
# ===================================================================
# Create static library target with dummy source
add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/dummy.cpp)

# Create an alias of the target with flexiv namespace
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  flexiv::flexiv_rdk
  coal::coal
  urdfdom::urdf_parser
  assimp::assimp
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
