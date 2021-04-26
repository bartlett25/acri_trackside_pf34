cmake_minimum_required(VERSION 3.0)

project(pose_cov_ops)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	mrpt_bridge
	roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(MRPT REQUIRED)
message(STATUS "Found MRPT: " ${MRPT_VERSION})
if("${MRPT_VERSION}" VERSION_LESS "1.9.9")
	# MRPT<2.0
	set (CMAKE_CXX_STANDARD 11)
	find_package(MRPT REQUIRED base)
else()
	# MRPT>=2.0
	find_package(MRPT REQUIRED poses)
endif()

catkin_package(
INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
		geometry_msgs
		mrpt_bridge
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}.cpp
)

if("${MRPT_VERSION}" VERSION_LESS "1.9.9")
    set(CMAKE_CXX_STANDARD 14)
else()
    #Need to enable C++17
    #We take into account that ubuntu 16.04 LTS only has cmake 3.5
    if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
        #Manually set c++17 which is not supported before cmake 3.8.0
        if (CMAKE_COMPILER_IS_GNUCXX)
            target_compile_options(${PROJECT_NAME} PUBLIC -std=gnu++17)
        elseif()
            message(FATAL_ERROR "You have to enable C++17 for your compiler here")
        endif()
    else()
        set(CMAKE_CXX_STANDARD 17)
    endif()
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		target_compile_options(${PROJECT_NAME} PUBLIC -Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${MRPT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )
