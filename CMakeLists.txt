cmake_minimum_required(VERSION 2.8.3)
project(ipa_rtabmap_interface)


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

FIND_PACKAGE(RTABMap REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)


find_package(Boost 1.58 REQUIRED COMPONENTS thread filesystem log system)


# Find Qt5 first
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
IF(NOT Qt5_FOUND)
  FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg)
ENDIF(NOT Qt5_FOUND)

# fix libproj.so not found on Xenial
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

find_package(catkin REQUIRED COMPONENTS
            dynamic_reconfigure
            cv_bridge
            image_geometry
            roscpp
            sensor_msgs
            visualization_msgs
            OpenCV
            pcl_ros
            rtabmap_ros
            )


generate_dynamic_reconfigure_options(
    cfg/dynReconfigure.cfg)



catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)


add_executable(ipa_rtabmap_interface src/ipa_rtabmap_interface.cpp)

target_link_libraries(ipa_rtabmap_interface ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}   ${PCL_LIBARIES}       ${Boost_LIBRARIES})

add_dependencies(ipa_rtabmap_interface ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS ipa_rtabmap_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
