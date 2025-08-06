# - Config file for the dlo package
# It defines the following variable
#  dlo_core_LIBRARIES - libraries to link against

include(CMakeFindDependencyMacro)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Find dependencies
find_dependency(Ceres REQUIRED)

# Our library dependencies (contains definitions for IMPORTED targets)
include("${CMAKE_CURRENT_LIST_DIR}/dlo_coreTargets.cmake")

# These are IMPORTED targets created by dlo_coreTargets.cmake
set(dlo_core_LIBRARIES dlo_core::dlo_core)
set(dlo_core_INCLUDE_DIRS /opt/pal/alum/include/dlo_core)
