cmake_minimum_required(VERSION 3.10)

# SpaceMouse hardware interface library
add_library(SPACEMOUSE SHARED
  src/spacemouse.cpp
)

target_include_directories(SPACEMOUSE PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(SPACEMOUSE
  hidapi-libusb
)

# Test executable
add_executable(test_spacemouse
  src/test_spacemouse.cpp
)

target_link_libraries(test_spacemouse
  SPACEMOUSE
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS SPACEMOUSE
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)