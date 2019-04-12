cmake_minimum_required(VERSION 2.8.3)
project(subt_gazebo_los_model_ign)

set(PKG_DEPENDS
  roscpp
  geometry_msgs
  subt_rf_interface_ign
)
set(BUILD_DEPENDS
  ${PKG_DEPENDS}
)
set(RUN_DEPENDS
  ${PKG_DEPENDS}
)

#######################################
## Find dependencies
#######################################

find_package(catkin REQUIRED ${BUILD_DEPENDS})

find_package(ignition-common3 REQUIRED)
find_package(ignition-msgs3 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-transport6 REQUIRED)

include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include  
  ${catkin_INCLUDE_DIRS}
)

#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  LIBRARIES subt_gazebo_los_model_ign
)

# Create the libVisibilityPlugin.so library.
set(visibility_plugin_name VisibilityPluginIgn)
add_library(${visibility_plugin_name}
  src/VisibilityPlugin.cc
  src/VisibilityTable.cc
  src/SimpleDOTParser.cc)
target_link_libraries(${visibility_plugin_name}
  ${catkin_LIBRARIES}
  ignition-common3::ignition-common3
)
install(TARGETS ${visibility_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(subt_gazebo_los_model_ign
  src/visibility_rf_model.cpp
  src/SimpleDOTParser.cc
  src/VisibilityTable.cc)
target_link_libraries(subt_gazebo_los_model_ign
  PUBLIC
  ${catkin_LIBRARIES}
  ignition-common3::ignition-common3
  ignition-math6::ignition-math6
  ignition-msgs3::ignition-msgs3
  ignition-transport6::ignition-transport6
)

install(TARGETS subt_gazebo_los_model_ign
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# catkin_add_gtest(simpleDOTParser test/simpleDOTParser.cc)
# target_link_libraries(simpleDOTParser
#   ${comms_broker_plugin_name}
# )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
