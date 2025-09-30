cmake_minimum_required(VERSION 3.5)
#######################
project(force_control_demo)

include_directories(
  include/
  yaml-cpp
  ${EIGEN3_INCLUDE_DIRS}
  /home/robotlab/ACP/libfranka-0.9.2/src/
  /home/robotlab/ACP/libfranka-0.9.2/include/franka/
  /home/robotlab/ACP/libfranka-0.9.2/common/include/
)

add_executable(ur_demo src/demo_with_ur.cc)
target_link_libraries(ur_demo
  HI_COMMON
  UR_RTDE
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
)

add_executable(coinft_demo src/demo_with_coinft.cc)
target_link_libraries(coinft_demo
  HI_COMMON
  UR_RTDE
  COIN_FT
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
)

add_executable(demo_with_franka src/demo_with_franka.cc)
target_link_libraries(demo_with_franka
  HI_COMMON
  FRANKA
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
  -lPocoNet
  -lPocoFoundation
)