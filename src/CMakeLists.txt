cmake_minimum_required(VERSION 3.22)
project(FreeFallObjectSimulation)
message("CMAKE CURRENT SOURCE DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
# add_subdirectory("${CMAKE_SOURCE_DIR}/matplotplusplus" "matplotplusplus/matplot")

add_library(FreeFallSim "")
target_sources(FreeFallSim
PRIVATE freefall_dragforce_simulation.cpp 
PUBLIC ${CMAKE_SOURCE_DIR}/include/freefall_dragforce_simulation.h)
target_include_directories(FreeFallSim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(FreeFallSim PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(FreeFallSim PUBLIC matplot)

add_executable(FreeFallUnderDragForceBall main.cpp)
target_link_libraries(FreeFallUnderDragForceBall PRIVATE FreeFallSim)
target_link_libraries(FreeFallUnderDragForceBall PRIVATE matplot)

