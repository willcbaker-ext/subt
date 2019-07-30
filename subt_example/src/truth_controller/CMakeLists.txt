find_package(ignition-common3 REQUIRED)
find_package(ignition-fuel_tools3 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(sdformat8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS graph)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(truth_controller_node
  EnvironmentGraph.cc
  truth_controller.cc
)
add_dependencies(truth_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(truth_controller_node
  PRIVATE
    ignition-common3::ignition-common3
    ignition-math6::ignition-math6
    ignition-fuel_tools3::ignition-fuel_tools3
    sdformat8::sdformat8
    Boost::graph
    ${catkin_LIBRARIES}
)

install(TARGETS truth_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
