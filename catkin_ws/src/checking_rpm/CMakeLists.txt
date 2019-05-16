cmake_minimum_required(VERSION 2.8.3)
project(checking_rpm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    arduino_msgs
    roscpp
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(updateRPM src/updateRPM.cpp)
target_link_libraries(updateRPM ${catkin_LIBRARIES})
add_dependencies(updateRPM ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS updateRPM
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


