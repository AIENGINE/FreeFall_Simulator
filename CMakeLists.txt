cmake_minimum_required(VERSION 3.22)
message("CMAKE VERSION: ${CMAKE_VERSION}")
project(FreeFallSimulations)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
  ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

enable_testing()
message("CMAKE SOURCE DIR: ${CMAKE_SOURCE_DIR}")
message("CMAKE PREFIX: ${CMAKE_PREFIX_PATH}")

add_subdirectory(external/matplotplusplus)

add_subdirectory(src)
add_subdirectory(test)

