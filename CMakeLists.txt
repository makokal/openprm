cmake_minimum_required (VERSION 2.4)
project (openprm)

set (openprm_VERSION_MAJOR 0)
set (openprm_VERSION_MINOR 2)

if( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
  set(PLUGIN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/plugins")
else( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
  set(PLUGIN_INSTALL_DIR "$ENV{PLUGIN_INSTALL_DIR}")
endif( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )

include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, please install openrave or check if openrave-config path is ok")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall -Wno-deprecated")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

include_directories(${OPENRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${OPENRAVE_LINK_DIRS})

add_library(openprm SHARED src/openprmmain.cpp src/classicprm.h src/prmparams.h src/prmplanning.h src/plugindefs.h )

set_target_properties(openprm PROPERTIES
                      LINK_FLAGS ${OPENRAVE_LINK_FLAGS}
                      COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})
                                                   
target_link_libraries(openprm ${OPENRAVE_LIBRARY} )

install(TARGETS openprm DESTINATION ${PLUGIN_INSTALL_DIR} )
