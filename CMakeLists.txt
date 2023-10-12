cmake_minimum_required(VERSION 3.10.0)
project(mgr_compression VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

include(CTest)
enable_testing()

find_package( OpenCV REQUIRED )
find_package( OpenMP REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(mgr_compression main.cpp)

target_link_libraries( mgr_compression PRIVATE OpenMP::OpenMP_CXX ${OpenCV_LIBS}  )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
