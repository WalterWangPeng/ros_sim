cmake_minimum_required(VERSION 2.8.3)
project(ayuan_gazebo)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(CATKIN_DEPENDS roscpp)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/lib)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
# install(DIRECTORY param/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
# )
# install(DIRECTORY launch/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
