# cmake requirements
CMAKE_MINIMUM_REQUIRED( VERSION 3.16 )

# project definition
PROJECT( liquid-edge-ocv-cam-streamer )
SET( VERSION 0.4.0 )

#set(CMAKE_VERBOSE_MAKEFILE ON)

# the modules
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/" )

# configuration
CONFIGURE_FILE( ${CMAKE_SOURCE_DIR}/cmake/config.h.cmake ${CMAKE_SOURCE_DIR}/src/config.h @ONLY )

# required packages
FIND_PACKAGE( Threads REQUIRED ) # threading needed
FIND_PACKAGE( OpenCV REQUIRED ) # opencv needed
FIND_PACKAGE( Sockpp REQUIRED ) # sockpp needed
SET( CMAKE_CXX_STANDARD 17 ) # C++17

# platform-dependant compilation options
IF( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT MINGW AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0.0" )
  SET ( EXTLIBS stdc++fs ) # Link stdc++fs when using gcc 8 in unix-like systems
ELSEIF( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
  SET( EXTLIBS ws2_32 ) # link winsock2 library
  SET( CMAKE_CXX_STANDARD 20 ) # the code is C++17, but MSVC requires C++20
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /UMBCS /D_UNICODE /DUNICODE" ) # force unicode
ENDIF()

# streaming transmitter
ADD_EXECUTABLE( le_ocv_streamer_tx src/le_ocv_streamer_tx.cpp )
TARGET_LINK_LIBRARIES( le_ocv_streamer_tx ${OpenCV_LIBS} Threads::Threads Sockpp::Sockpp ${EXTLIBS} )
SET_TARGET_PROPERTIES( le_ocv_streamer_tx PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

# streaming receiver
ADD_EXECUTABLE( le_ocv_streamer_rx src/le_ocv_streamer_rx.cpp )
TARGET_LINK_LIBRARIES( le_ocv_streamer_rx ${OpenCV_LIBS} Threads::Threads Sockpp::Sockpp ${EXTLIBS} )
SET_TARGET_PROPERTIES( le_ocv_streamer_rx PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

# tell CMake to use CTest extension
ENABLE_TESTING()

# simple tests
ADD_TEST(NAME print_help_tx COMMAND le_ocv_streamer_tx -h )
ADD_TEST(NAME print_help_rx COMMAND le_ocv_streamer_rx -h )

# install the programs and auxiliary files
INSTALL( TARGETS le_ocv_streamer_tx le_ocv_streamer_rx RUNTIME )
INSTALL( DIRECTORY "${CMAKE_SOURCE_DIR}/../liquid-edge-model-zoo/Caffe" TYPE DATA )
INSTALL( DIRECTORY "${CMAKE_SOURCE_DIR}/../liquid-edge-model-zoo/Darknet" TYPE DATA )

# uninstall the programs
IF( NOT TARGET uninstall )
  CONFIGURE_FILE( "${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY )
  ADD_CUSTOM_TARGET( uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake )
ENDIF()

set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_IGNORE_FILES
  \\.git/
  build/
  ".*~$"
)
set(CPACK_VERBATIM_VARIABLES YES)
include(CPack)