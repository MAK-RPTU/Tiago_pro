#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dlo_core::dlo_core" for configuration "None"
set_property(TARGET dlo_core::dlo_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(dlo_core::dlo_core PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdlo_core.so"
  IMPORTED_SONAME_NONE "libdlo_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dlo_core::dlo_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_dlo_core::dlo_core "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdlo_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
