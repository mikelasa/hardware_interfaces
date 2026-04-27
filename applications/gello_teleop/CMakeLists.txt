cmake_minimum_required(VERSION 3.5)
project(gello_teleop)

add_executable(gello_teleop src/gello_teleop.cc)

target_include_directories(gello_teleop PRIVATE
  $ENV{HOME}/ACP/libfranka-0.9.2/src/
  $ENV{HOME}/ACP/libfranka-0.9.2/include/franka/
  $ENV{HOME}/ACP/libfranka-0.9.2/common/include/
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_INSTALL_PREFIX}/include
)

target_link_libraries(gello_teleop
  HI_COMMON
  FRANKA
  GELLO
  ${FORCE_CONTROLLERS}
  ${RUT}
  ${YAMLLib}
  -lPocoNet
  -lPocoFoundation
)

install(TARGETS gello_teleop
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
)