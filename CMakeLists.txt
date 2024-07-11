cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

set(DCMAKE_PREFIX_PATH "/home/jonghun/Workspace/raisimLib-master/raisim/linux/lib/cmake/raisim" CACHE PATH "RaiSim installation path")

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories (${EIGEN3_INCLUDE_DIRS})

add_executable(DHAL_test main.cpp)
add_executable(DHAL_test2 humanoid.cpp)

target_link_libraries(DHAL_test PUBLIC raisim::raisim)
target_link_libraries(DHAL_test2 PUBLIC raisim::raisim)

if(NOT WIN32)
  target_link_libraries(DHAL_test PUBLIC pthread)
  target_link_libraries(DHAL_test2 PUBLIC pthread)
else()
  target_link_libraries(DHAL_test PUBLIC Ws2_32 Winmm)
  target_link_libraries(DHAL_test2 PUBLIC Ws2_32 Winmm)
endif()

target_include_directories(DHAL_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(DHAL_test2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
