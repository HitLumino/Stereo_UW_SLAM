cmake_minimum_required(VERSION 2.8)
project(STEREO_UW_SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  #SET(CMAKE_BUILD_TYPE Release)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

#CUDA
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

IF(WIN32) # Windows
    if (CMAKE_CL_64)
        SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
        SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
        SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
    else()
        message(FATAL_ERROR "You've selected the 32bit version of ${CMAKE_GENERATOR}. \n Please delete the cache (file->Delete Cache) and use the 64bit version. (${CMAKE_GENERATOR} Win64)")
    endif()
ELSE() # Linux
    find_package(ZED 2.1 REQUIRED)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-std=c++0x -O3 -pthread)
ENDIF(WIN32)

#ZED

include_directories(${ZED_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cpp
src/Tracking.cpp
src/LocalMapping.cpp
src/LoopClosing.cpp
src/ORBextractor.cpp
src/ORBmatcher.cpp
src/FrameDrawer.cpp
src/Converter.cpp
src/MapPoint.cpp
src/KeyFrame.cpp
src/Map.cpp
src/MapDrawer.cpp
src/Optimizer.cpp
src/PnPsolver.cpp
src/Frame.cpp
src/KeyFrameDatabase.cpp
src/Sim3Solver.cpp
src/Initializer.cpp
src/Viewer.cpp
#src/Lsd_line.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build examples

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cpp)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cpp)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cpp)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Zed)

#add_executable(zed_orb
#Examples/Zed/run_zed.cpp)
#target_link_libraries(zed_orb ${PROJECT_NAME})
