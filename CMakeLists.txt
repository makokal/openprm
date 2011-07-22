cmake_minimum_required (VERSION 2.6.0)
project(openprm)
find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )


if( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
  set(PLUGIN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/plugins")
else( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
  set(PLUGIN_INSTALL_DIR "$ENV{PLUGIN_INSTALL_DIR}")
endif( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )


# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(openprm SHARED openprm.cpp
    #planners/classicprm.h
    planners/prmparams.h
    planners/samplerbase.h
    #planners/sblplanner2.h
    #planners/visprm.h
    planners/prmplanning.h
    planners/roadmap_inspector.h
    utils/oputils.h
    #utils/spatialrep.h
    utils/prmgraph.h plugindefs.h)
set_target_properties(openprm PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(openprm ${OpenRAVE_LIBRARIES})
install(TARGETS openprm DESTINATION ${PLUGIN_INSTALL_DIR} )
