cmake_minimum_required(VERSION 2.8.3)
project(gpio_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}/include
)
find_library(wiringPi_LIB wiringPi)
add_executable(gpio_tester src/test_gpio.cpp)
target_link_libraries(gpio_tester ${catkin_LIBRARIES} ${wiringPi_LIB})
add_dependencies(gpio_tester gpio_test_gencpp)
