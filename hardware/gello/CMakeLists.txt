cmake_minimum_required(VERSION 3.10)

find_library(DXL_LIBRARY dxl_x64_cpp
  HINTS
    $ENV{HOME}/.local/lib
    /usr/local/lib
    /usr/lib
)
find_path(DXL_INCLUDE_DIR dynamixel_sdk/dynamixel_sdk.h
  HINTS
    $ENV{HOME}/.local/include
    /usr/local/include
    /usr/include
)

if(NOT DXL_LIBRARY OR NOT DXL_INCLUDE_DIR)
  message(FATAL_ERROR "DynamixelSDK C++ library not found. "
    "Build and install it from gello_software/third_party/DynamixelSDK/c++/build/linux64")
endif()

add_library(GELLO SHARED
  src/gello.cpp
)

target_include_directories(GELLO PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include
  ${DXL_INCLUDE_DIR}
)

target_link_libraries(GELLO
  ${DXL_LIBRARY}
)

add_executable(test_gello
  src/gello_test.cpp
)

target_link_libraries(test_gello
  GELLO
  ${DXL_LIBRARY}
  ${YAMLLib}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS GELLO
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)