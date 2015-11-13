cmake_minimum_required(VERSION 2.8.3)
project(tf2_eigen)

add_definitions(-std=gnu++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs tf2_ros_utils
  DEPENDS eigen)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS tf2 tf2_ros_utils)
  include_directories(include ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})

  catkin_add_gtest(tf2_eigen-test test/tf2_eigen-test.cpp)
  target_link_libraries(tf2_eigen-test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()
