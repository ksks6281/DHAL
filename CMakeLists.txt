cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

# set(DCMAKE_PREFIX_PATH "/home/jonghun/Workspace/raisimLib/raisim/linux" CACHE PATH "RaiSim installation path")

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories (${EIGEN3_INCLUDE_DIRS})

add_executable(DHAL_main main.cpp)
add_executable(DHAL_humanoid humanoid.cpp)
add_executable(DHAL_4bar 4bar_test_1.cpp)

target_link_libraries(DHAL_main PUBLIC raisim::raisim)
target_link_libraries(DHAL_humanoid PUBLIC raisim::raisim)
target_link_libraries(DHAL_4bar PUBLIC raisim::raisim)

if(NOT WIN32)
  target_link_libraries(DHAL_main PUBLIC pthread)
  target_link_libraries(DHAL_humanoid PUBLIC pthread)
  target_link_libraries(DHAL_4bar PUBLIC pthread)
else()
  target_link_libraries(DHAL_main PUBLIC Ws2_32 Winmm)
  target_link_libraries(DHAL_humanoid PUBLIC Ws2_32 Winmm)
  target_link_libraries(DHAL_4bar PUBLIC Ws2_32 Winmm)
endif()

target_include_directories(DHAL_main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(DHAL_humanoid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(DHAL_4bar PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
