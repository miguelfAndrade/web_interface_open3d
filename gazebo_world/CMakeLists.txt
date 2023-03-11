cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)


link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(velodyne_plugin SHARED velodyne_plugin.cc)
target_link_libraries(velodyne_plugin velodyne_plugin_msgs ${GAZEBO_LIBRARIES})
add_dependencies(velodyne_plugin velodyne_plugin_msgs)

# Build the stand-alone test program
add_executable(vel vel.cc)
add_executable(vel_data vel_data.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
  target_link_libraries(vel_data ${GAZEBO_LIBRARIES} pthread)
else()
  target_link_libraries(vel ${GAZEBO_LIBRARIES})
  target_link_libraries(vel_data ${GAZEBO_LIBRARIES})
endif()