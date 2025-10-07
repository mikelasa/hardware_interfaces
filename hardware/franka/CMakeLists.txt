include_directories(
  include/
  /home/robotlab/ACP/libfranka-0.9.2/src/
  /home/robotlab/ACP/libfranka-0.9.2/include/franka/
  /home/robotlab/ACP/libfranka-0.9.2/common/include/
  /usr/include/eigen3
)

find_package(Franka REQUIRED)

add_library(FRANKA SHARED src/franka.cpp)
target_link_libraries(FRANKA ${RUT} ${YAMLLib} Franka::Franka
)

target_include_directories(FRANKA PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# test
add_executable(test_cases src/test_cases.cpp)
add_executable(test_franka src/test_franka.cpp)

target_link_libraries(
  test_cases FRANKA
  ${RUT}
  ${TIMER_LIB}
  Franka::Franka
  -lPocoNet
  -lPocoFoundation
  )

target_link_libraries(
  test_franka FRANKA
  ${RUT}
  ${TIMER_LIB}
  Franka::Franka
  -lPocoNet
  -lPocoFoundation
  )

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS FRANKA
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)