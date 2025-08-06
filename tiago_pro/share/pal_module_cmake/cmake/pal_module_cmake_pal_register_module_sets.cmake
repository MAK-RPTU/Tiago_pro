# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

if(PAL_MODULE_CMAKE_PAL_REGISTER_MODULE_SETS_CMAKE_GUARD)
  return()
endif()
set(PAL_MODULE_CMAKE_PAL_REGISTER_MODULE_SETS_CMAKE_GUARD TRUE)

macro(pal_register_module_sets)
  set(relative_module_set_filenames ${ARGN})

  foreach(relative_module_set_filename ${relative_module_set_filenames})
    set(abs_module_set_filename "${CMAKE_CURRENT_SOURCE_DIR}/${relative_module_set_filename}")
    if(NOT EXISTS "${abs_module_set_filename}")
      message(FATAL_ERROR "Given module set file '${abs_module_set_filename}' does not exist")
    endif()

    set(relative_module_set_dir "")
    get_filename_component(relative_module_set_dir "${relative_module_set_filename}" DIRECTORY)
    install(FILES ${relative_module_set_filename} DESTINATION share/${PROJECT_NAME}/${relative_module_set_dir})
  endforeach()

  ament_index_register_resource(pal_system_module_set CONTENT ${relative_module_set_filenames})
endmacro()