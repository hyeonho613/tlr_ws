#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "v4l2_camera::v4l2_camera" for configuration "Release"
set_property(TARGET v4l2_camera::v4l2_camera APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4l2_camera::v4l2_camera PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4l2_camera.so"
  IMPORTED_SONAME_RELEASE "libv4l2_camera.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4l2_camera::v4l2_camera )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4l2_camera::v4l2_camera "${_IMPORT_PREFIX}/lib/libv4l2_camera.so" )

# Import target "v4l2_camera::v4l2_camera_node" for configuration "Release"
set_property(TARGET v4l2_camera::v4l2_camera_node APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4l2_camera::v4l2_camera_node PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/v4l2_camera/v4l2_camera_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4l2_camera::v4l2_camera_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4l2_camera::v4l2_camera_node "${_IMPORT_PREFIX}/lib/v4l2_camera/v4l2_camera_node" )

# Import target "v4l2_camera::v4l2_camera_compose_test" for configuration "Release"
set_property(TARGET v4l2_camera::v4l2_camera_compose_test APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4l2_camera::v4l2_camera_compose_test PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/v4l2_camera/v4l2_camera_compose_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4l2_camera::v4l2_camera_compose_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4l2_camera::v4l2_camera_compose_test "${_IMPORT_PREFIX}/lib/v4l2_camera/v4l2_camera_compose_test" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
