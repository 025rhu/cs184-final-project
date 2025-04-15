cmake_minimum_required(VERSION 3.14) 
project(final_project)

set(CMAKE_CXX_STANDARD 17) 

include(FetchContent)

FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG v5.4.3
)
FetchContent_MakeAvailable(assimp)



add_executable(final_project main.cpp file1.cpp)  


target_link_libraries(final_project PRIVATE assimp)
