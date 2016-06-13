
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

include_directories(include
  ${USE_OROCOS_INCLUDE_DIRS}
  ${RTTToolchainTools_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
  )

# Creates a component library libmyrobot-<target>.so
# and installs in the directory lib/orocos/myrobot/
#
orocos_component(${CMAKE_PROJECT_NAME} controller/rtt-controller-template.cpp)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/myrobot/ directory
#
# orocos_install_headers(rtt-controller-template.hpp)
#
# You may add multiple orocos_install_headers statements.
target_link_libraries(${CMAKE_PROJECT_NAME} ${RTTToolchainTools_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TINYXML_LIBRARIES} ${RST-RT_LIBRARIES} ${RTT-CORE-EXTENSIONS_LIBRARIES})
