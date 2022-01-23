include( GNUInstallDirs )

find_path( SOCKPP_INCLUDE_DIR NAMES sockpp HINTS "C:/Program Files (x86)/sockpp/include" )
if( WIN32 )
  find_library( SOCKPP_LIBRARY NAMES sockpp-static HINTS "C:/Program Files (x86)/sockpp/lib" )
else()
  find_library( SOCKPP_LIBRARY NAMES sockpp HINTS )
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Sockpp DEFAULT_MSG SOCKPP_LIBRARY SOCKPP_INCLUDE_DIR )

mark_as_advanced( SOCKPP_LIBRARY SOCKPP_INCLUDE_DIR )

if( SOCKPP_FOUND AND NOT TARGET Sockpp::Sockpp )
if( WIN32 )
  add_library( Sockpp::Sockpp STATIC IMPORTED )
else()
  add_library( Sockpp::Sockpp SHARED IMPORTED )
endif()
set_target_properties( Sockpp::Sockpp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${SOCKPP_INCLUDE_DIR}" IMPORTED_LOCATION ${SOCKPP_LIBRARY} )
endif()
