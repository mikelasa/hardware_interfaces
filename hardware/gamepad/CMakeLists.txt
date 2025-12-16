cmake_minimum_required(VERSION 3.10)

# Xbox Gamepad hardware interface library using Linux Joystick API
add_library(GAMEPAD SHARED
  src/gamepad.cpp
)

target_include_directories(GAMEPAD PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(GAMEPAD
  HI_COMMON
)

# Test executable
add_executable(test_gamepad
  src/gamepad_test.cpp
)

target_link_libraries(test_gamepad
  GAMEPAD
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS GAMEPAD
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)
