# Copyright: (C) 2010 RobotCub Consortium
# Author: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 3.0)
# find YARP
find_package(YARP REQUIRED )
find_package(AMOR_API)
# set up our program
add_executable(amor cartesian_rate_AMORfinal.cpp)
# link with YARP libraries
target_link_libraries(final ${YARP_LIBRARIES})
target_link_libraries(final ${AMOR_API_LIBRARIES})
add_definitions(-DUNIX)
