cmake_minimum_required(VERSION 3.5)

# https://github.com/wjwwood/serial - https://github.com/lunarifish/serial
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/serial)

add_library(third_party INTERFACE)
target_link_libraries(third_party INTERFACE serial)