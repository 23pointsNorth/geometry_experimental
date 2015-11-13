cmake_minimum_required(VERSION 2.8.3)
project(tf2_geometry_msgs)

add_definitions(-std=gnu++11)

find_package(orocos_kdl)
find_package(catkin REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

# Issue #53
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

catkin_package(
   LIBRARIES ${KDL_LIBRARY}
   INCLUDE_DIRS include
   DEPENDS orocos_kdl
   CATKIN_DEPENDS geometry_msgs tf2)


include_directories(include
                    ${catkin_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_python_setup()
