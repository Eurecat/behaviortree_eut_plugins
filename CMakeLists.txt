cmake_minimum_required(VERSION 3.8)
project(behaviortree_eut_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED) # Ensure this is findable

# Include directories
include_directories(
  include
)

############### EUT Modifications ####################
list(APPEND BT4_EUT_PLUGINS_SRC
    src/eut_basic_types.cpp
    src/utils/eut_utils.cpp
    src/eut_debug.cpp
    src/eut_factory.cpp
    src/loggers/bt_file_logger.cpp
    src/loggers/bt_zmq_publisher.cpp
)
######################################################

# Create a library for the enhancements
add_library(${PROJECT_NAME}_lib SHARED ${BT4_EUT_PLUGINS_SRC})

ament_target_dependencies(${PROJECT_NAME}_lib behaviortree_cpp)

# Specify target properties
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${behaviortree_cpp_INCLUDE_DIRS}> # Ensure it exposes the library's headers
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Build nodemodels library
add_library(bt_eut_nodemodels_plugin SHARED src/plugin.cpp src/eut_basic_types.cpp)
target_include_directories(bt_eut_nodemodels_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(bt_eut_nodemodels_plugin PRIVATE  BT_PLUGIN_EXPORT )
target_link_libraries(bt_eut_nodemodels_plugin ${PROJECT_NAME}_lib) 
ament_target_dependencies(bt_eut_nodemodels_plugin behaviortree_cpp)

install(TARGETS
  bt_eut_nodemodels_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

add_executable(bt_plugins_to_xml         src/tools/bt_plugins_to_xml.cpp )
include_directories(${behaviortree_cpp_INCLUDE_DIRS})
ament_target_dependencies(bt_plugins_to_xml behaviortree_cpp)
# target_link_options(bt_plugins_to_xml PUBLIC -rdynamic)
install(TARGETS bt_plugins_to_xml
        DESTINATION lib/${PROJECT_NAME} )


# Add an executable example (if any)
add_executable(01_basic_example src/examples/01_basic.cpp) # Replace main.cpp with the actual executable file
target_link_libraries(01_basic_example ${PROJECT_NAME}_lib)
ament_target_dependencies(01_basic_example ament_index_cpp)

# Install the examples folder into the share directory
install(
  DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
)
  
# Install executables
install(
  TARGETS 01_basic_example
  DESTINATION lib/${PROJECT_NAME}
)

# Export the package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(behaviortree_cpp)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
