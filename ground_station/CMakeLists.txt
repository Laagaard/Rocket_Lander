cmake_minimum_required(VERSION 3.29) # minimum version of CMake required
project(ground_station VERSION 1.0) # project name and version

set(wxBUILD_SHARED OFF) # only build wxWidgets static libraries
add_subdirectory(libs/wxWidgets) # add wxWidgets as project subdirectory
add_executable(ground_station ground_station.cpp) # project executable
target_link_libraries(ground_station wx::net wx::core wx::base) # link relevant wxWidgets libraries