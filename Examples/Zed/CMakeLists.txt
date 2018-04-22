# CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

add_definitions(-std=c++0x)# -m64)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" ) # Release Perf mode 

# SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 2.1 REQUIRED)

find_package(CUDA 8.0 REQUIRED)
find_package(OpenCV 3.2 COMPONENTS core highgui imgproc REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

add_executable( zed_orb run_zed.cpp )
TARGET_LINK_LIBRARIES( zed_orb
                        ${ZED_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${SPECIAL_OS_LIBS}#这个不加编译不了
                        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
                        ${PROJECT_NAME}
                    )


