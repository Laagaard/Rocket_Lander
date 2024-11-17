set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include") # path to include directory
include_directories(PRIVATE ${INCLUDE_DIR}) # add include directory

file(GLOB_RECURSE SOURCE_FILES "*.cpp" PARENT_SCOPE) # recursively glob all source files
add_executable(ground_station ${SOURCE_FILES}) # project executable