project(jpointcloud)
cmake_minimum_required(VERSION 2.8)
    
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(OS_PATH "linux")
    set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread -pthread")
else()
   set (OS_PATH "osx")
endif()

set (BOOST_ROOT "/usr/")
set (BOOST_INCLUDE_DIR "/usr/include/boost")
set (BOOST_LIB_DIR "/usr/local/lib")

set (PCL_ROOT "/jaunt/users/alfred/dev/pcl")
set (PCL_INCLUDE_DIR "${PCL_ROOT}/include/pcl-1.8")
set (PCL_LIB_DIR "${PCL_ROOT}/lib")

set (OPENEXR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set (OPENEXR_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set (OPENEXR_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")

set (EIGEN_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set (EIGEN_INCLUDE_DIR "${EIGEN_ROOT_DIR}/include/eigen3")

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED)

set(GCC_INCLUDE_DIRS "${PCL_INCLUDE_DIR}"
                     "${EIGEN_INCLUDE_DIR}"
                     "${OPENEXR_INCLUDE_DIR}"
                     "${BOOST_INCLUDE_DIR}")
                  
set(GCC_LINK_DIRS "${PCL_LIB_DIR}" 
                  "${OPENEXR_LIB_DIR}"
                  "${GENERIC_LIB_DIRS}"
                  "${BOOST_LIB_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${OS_PATH}/bin")

file(GLOB SRC "*.cpp" "src/*.cpp")
set(BUILD_SHARED_LIBS OFF)

set(LIBS boost_system boost_filesystem gomp 
         Half IlmImfUtil IlmImf IlmThread Iex IexMath Imath z 
         pcl_search pcl_filters pcl_common pcl_io pcl_io_ply pcl_surface 
         pcl_kdtree pcl_registration pcl_sample_consensus pcl_features 
         boost_filesystem boost_system)

# set(LIBS gomp Half z)


include_directories(${GCC_INCLUDE_DIRS})
link_directories(${GCC_LINK_DIRS} ${GENERIC_LIB_DIRS})

# print some useful info
#message( STATUS "OS_PATH: " ${OS_PATH} )
#message( STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR} )
#message( STATUS "CMAKE_LIBRARY_PATH: " ${CMAKE_LIBRARY_PATH} )
#message( STATUS "CMAKE_INCLUDE_PATH: " ${CMAKE_INCLUDE_PATH} )
#message( STATUS "GCC_INCLUDE_DIRS: " ${GCC_INCLUDE_DIRS} )
#message( STATUS "link dir for openexr: " ${CMAKE_CURRENT_SOURCE_DIR}/${OPENEXR_PATH}/lib )

#add_definitions(${PCL_DEFINITIONS})
aux_source_directory(. SRC)
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${LIBS})