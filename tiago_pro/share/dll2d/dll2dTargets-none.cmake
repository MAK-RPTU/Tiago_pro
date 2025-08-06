#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dll2d::dll2d" for configuration "None"
set_property(TARGET dll2d::dll2d APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(dll2d::dll2d PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdll2d.so"
  IMPORTED_SONAME_NONE "libdll2d.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dll2d::dll2d )
list(APPEND _IMPORT_CHECK_FILES_FOR_dll2d::dll2d "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdll2d.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
