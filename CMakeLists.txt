cmake_minimum_required(VERSION 3.10)
project(Drone_Mapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(Drone_Mapper main.cpp "Mapper.cpp")
target_link_libraries(Drone_Mapper PRIVATE ${OpenCV_LIBS})