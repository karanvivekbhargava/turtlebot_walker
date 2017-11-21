cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_walker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(walker src/main.cpp src/walker.cpp)

target_link_libraries(walker
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    walkerTest
    test/walkerTest.launch
    test/main.cpp
    test/walkerTest.cpp
    src/walker.cpp
  )

  target_link_libraries(
    walkerTest
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    walkerTest
    walker
    ${catkin_EXPORTED_TARGETS}
  )
endif()
