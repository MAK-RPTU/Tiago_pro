# - Config file for the dll2d package
# It defines the following variable
#  dll2d_LIBRARIES - libraries to link against

include(CMakeFindDependencyMacro)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Find dependencies
find_dependency(Ceres REQUIRED)

# Our library dependencies (contains definitions for IMPORTED targets)
include("${CMAKE_CURRENT_LIST_DIR}/dll2dTargets.cmake")

# These are IMPORTED targets created by dll2dTargets.cmake
set(dll2d_LIBRARIES dll2d::dll2d)
set(dll2d_INCLUDE_DIRS /opt/pal/alum/include/dll2d)
