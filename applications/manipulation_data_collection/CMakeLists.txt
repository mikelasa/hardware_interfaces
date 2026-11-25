cmake_minimum_required(VERSION 3.5)
#######################
project(manipulation_data_collection)

find_package(OpenCV REQUIRED)

include_directories(
  include/
  yaml-cpp
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  $ENV{HOME}/ACP/libfranka-0.9.2/src/
  $ENV{HOME}/ACP/libfranka-0.9.2/include/franka/
  $ENV{HOME}/ACP/libfranka-0.9.2/common/include/
)

add_executable(main_teleop src/main_teleop.cc)
target_link_libraries(main_teleop
  ManipServer
  SPACEMOUSE
  ${RUT}
  ${YAMLLib}
  ${OpenCV_LIBS}
  -lPocoNet
  -lPocoFoundation
  pthread
)
