PROJECT(BayesianTracking)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

add_definitions(-std=c++11)

find_package(Qt5Core REQUIRED)

FIND_PACKAGE(MRPT REQUIRED base;bayes;gui)

ADD_EXECUTABLE(bayesiantracking
    TemplateInst.cpp
    test.cpp
    KFTracking.cpp
    )

TARGET_LINK_LIBRARIES(bayesiantracking
    ${MRPT_LIBS}
	)

qt5_use_modules(bayesiantracking Core)

# Set optimized building:
# IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
# ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
