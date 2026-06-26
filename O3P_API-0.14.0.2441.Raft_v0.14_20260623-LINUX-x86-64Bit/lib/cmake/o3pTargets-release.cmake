#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "o3p::o3p" for configuration "Release"
set_property(TARGET o3p::o3p APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(o3p::o3p PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libo3p.so"
  IMPORTED_SONAME_RELEASE "libo3p.so"
  )

list(APPEND _cmake_import_check_targets o3p::o3p )
list(APPEND _cmake_import_check_files_for_o3p::o3p "${_IMPORT_PREFIX}/lib/libo3p.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
