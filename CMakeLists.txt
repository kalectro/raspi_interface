cmake_minimum_required(VERSION 2.8.3)
project(raspi_interface)

find_package(catkin REQUIRED COMPONENTS roscpp bosch_drivers_common gpio_driver)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES raspi_interface
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(raspi_interface src/raspi_interface.cpp)

target_link_libraries(raspi_interface ${catkin_LIBRARIES})

install(TARGETS raspi_interface
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
