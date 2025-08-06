#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "scan_quality_estimator::scan_quality_estimator" for configuration "None"
set_property(TARGET scan_quality_estimator::scan_quality_estimator APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(scan_quality_estimator::scan_quality_estimator PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libscan_quality_estimator.so"
  IMPORTED_SONAME_NONE "libscan_quality_estimator.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS scan_quality_estimator::scan_quality_estimator )
list(APPEND _IMPORT_CHECK_FILES_FOR_scan_quality_estimator::scan_quality_estimator "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libscan_quality_estimator.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
