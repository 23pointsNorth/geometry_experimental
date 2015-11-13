cmake_minimum_required(VERSION 2.8.3)
project(tf2_sensor_msgs)

add_definitions(-std=gnu++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Eigen)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS sensor_msgs tf2_ros_utils
   DEPENDS Eigen
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)

find_package(catkin REQUIRED COMPONENTS rostest sensor_msgs tf2_ros tf2_ros_utils)

include_directories(include ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(test_tf2_sensor_msgs_cpp EXCLUDE_FROM_ALL test/test_tf2_sensor_msgs.cpp)
target_link_libraries(test_tf2_sensor_msgs_cpp ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


if(TARGET tests)
  add_dependencies(tests test_tf2_sensor_msgs_cpp)
endif()
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)


endif()
