cmake_minimum_required (VERSION 2.6.0)
project(openprm)
find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} REQUIRED)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})


add_library(openprm SHARED src/openprm.cpp)
set_target_properties(openprm PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(openprm ${OpenRAVE_LIBRARIES})
install(TARGETS openprm DESTINATION .)
