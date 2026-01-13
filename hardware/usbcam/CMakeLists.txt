find_package(OpenCV REQUIRED)

include_directories(
  include/
)

add_executable(USBRESET2 src/usbreset.cpp)

add_executable(TEST2 src/test.cpp)
target_link_libraries(TEST2 ${OpenCV_LIBS}
)

add_library(usbcam SHARED src/usbcam.cpp)
target_link_libraries(usbcam ${OpenCV_LIBS})

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS usbcam
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)