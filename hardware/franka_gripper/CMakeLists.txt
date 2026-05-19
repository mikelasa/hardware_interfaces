include_directories(
  include/
  $ENV{HOME}/ACP/libfranka-0.9.2/include/franka/
  $ENV{HOME}/ACP/libfranka-0.9.2/common/include/
)

find_package(Franka REQUIRED)

add_library(FRANKA_GRIPPER SHARED
  src/franka_gripper.cpp
)

target_link_libraries(FRANKA_GRIPPER
  ${RUT}
  ${YAMLLib}
  Franka::Franka
  pthread
)

target_include_directories(FRANKA_GRIPPER PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS FRANKA_GRIPPER
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)
