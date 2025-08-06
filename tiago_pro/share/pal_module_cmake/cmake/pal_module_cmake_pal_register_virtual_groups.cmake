# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

if(PAL_MODULE_CMAKE_PAL_REGISTER_VIRTUAL_GROUPS_CMAKE_GUARD)
  return()
endif()
set(PAL_MODULE_CMAKE_PAL_REGISTER_VIRTUAL_GROUPS_CMAKE_GUARD TRUE)

macro(pal_register_virtual_groups)
  set(relative_virtual_group_filenames ${ARGN})

  foreach(relative_virtual_group_filename ${relative_virtual_group_filenames})
    set(abs_virtual_group_filename "${CMAKE_CURRENT_SOURCE_DIR}/${relative_virtual_group_filename}")
    if(NOT EXISTS "${abs_virtual_group_filename}")
      message(FATAL_ERROR "Given virtual group file '${abs_virtual_group_filename}' does not exist")
    endif()

    set(relative_virtual_group_dir "")
    get_filename_component(relative_virtual_group_dir "${relative_virtual_group_filename}" DIRECTORY)
    install(FILES ${relative_virtual_group_filename} DESTINATION share/${PROJECT_NAME}/${relative_virtual_group_dir})
  endforeach()

  ament_index_register_resource(pal_system_virtual_group CONTENT ${relative_virtual_group_filenames})
endmacro()