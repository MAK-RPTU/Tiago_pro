# - Config file for the scan_quality_estimator package
# It defines the following variable
#  scan_quality_estimator_LIBRARIES - libraries to link against

include(CMakeFindDependencyMacro)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Find dependencies
find_dependency(Eigen3 REQUIRED)

# Our library dependencies (contains definitions for IMPORTED targets)
include("${CMAKE_CURRENT_LIST_DIR}/scan_quality_estimatorTargets.cmake")

# These are IMPORTED targets created by scan_quality_estimatorTargets.cmake
set(scan_quality_estimator_LIBRARIES scan_quality_estimator::scan_quality_estimator)
set(scan_quality_estimator_INCLUDE_DIRS /opt/pal/alum/include/scan_quality_estimator)
