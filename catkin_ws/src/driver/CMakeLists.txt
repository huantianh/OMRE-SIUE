cmake_minimum_required(VERSION 2.8.3)
project(driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager
    arduino_msgs
    std_msgs
    serial
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(arduino_listener src/arduino_listener.cpp src/ArduinoInterface.cpp)
add_dependencies(arduino_listener ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(arduino_listener ${catkin_LIBRARIES})

add_executable(arduino_commander src/arduino_commander.cpp src/ArduinoInterface.cpp)
add_dependencies(arduino_commander ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(arduino_commander ${catkin_LIBRARIES})


install(TARGETS arduino_listener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS arduino_commander
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


