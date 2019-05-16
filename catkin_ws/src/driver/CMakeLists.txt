cmake_minimum_required(VERSION 2.8.3)
project(driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager
    arduino_msgs
    serial
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(arduino_driver src/arduino_driver.cpp src/ArduinoInterface.cpp)
add_dependencies(arduino_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(arduino_driver 
    ${catkin_LIBRARIES}
)

install(TARGETS arduino_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


