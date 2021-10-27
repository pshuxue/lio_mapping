cmake_minimum_required(VERSION 3.0.0)
project(lio_mapping VERSION 0.1.0)


#eigen3
include_directories( "/usr/include/eigen3" )

#glog
find_package(Glog REQUIRED)
include_directories( ${Glog_INCLUDE_DIRS})

#gtest
find_package(GTest REQUIRED)
include_directories( ${GTest_INCLUDE_DIRS})

#gflags
find_package(GFlags REQUIRED)
include_directories( ${GFlags_INCLUDE_DIRS})

#ceres
find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


set( THIRD_PARTY_LIBS
        ${GLOG_LIBRARIES}
        ${GTEST_LIBRARIES}
        ${GTEST_MAIN_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
)

set( SRC_LIST
        src/imu/imu_tracker.cpp
)

add_library(lio_lib ${SRC_LIST})
target_link_libraries(lio_lib ${THIRD_PARTY_LIBS})

include_directories(src)

add_subdirectory(src)
add_subdirectory(app)