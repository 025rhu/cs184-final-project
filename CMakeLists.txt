cmake_minimum_required(VERSION 2.8)
project(FinalProject)

if(APPLE)
    add_definitions(-DGL_SILENCE_DEPRECATION)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#-------------------------------------------------------------------------------
# Build options
#-------------------------------------------------------------------------------
option(BUILD_DEBUG "Build with debug settings" OFF)
option(BUILD_DOCS  "Build documentation"       OFF)

if (BUILD_DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  set(BUILD_DEBUG ON CACHE BOOL "Build with debug settings" FORCE)
endif()

#-------------------------------------------------------------------------------
# Platform-specific settings
#-------------------------------------------------------------------------------

###################
# Building on OSX #
###################
if(APPLE)
  # OSX Framework dependencies
  include_directories("/System/Library/Frameworks")
  find_library(COCOA_LIBRARIES Cocoa)
  find_library(IOKIT_LIBRARIES IOkit)
  find_library(COREVIDEO_LIBRARIES CoreVideo)

  # Compiler settings
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(CLANG_CXX_FLAGS "-std=c++11 -m64")
    if(BUILD_DEBUG)
      set(CLANG_CXX_FLAGS "${CLANG_CXX_FLAGS} -g")
    else()
      set(CLANG_CXX_FLAGS "${CLANG_CXX_FLAGS} -O3 -funroll-loops -Wno-narrowing -Wno-deprecated-register")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CLANG_CXX_FLAGS}")
  endif()

  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(GCC_CXX_FLAGS "-std=gnu++11 -m64")
    if(BUILD_DEBUG)
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -g")
    else()
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -O3 -fopenmp")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_CXX_FLAGS}")
  endif()
endif()

##################
# Building on Linux
##################
if(UNIX AND NOT APPLE)
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(GCC_CXX_FLAGS "-std=gnu++11 -m64 -Wno-deprecated-declarations -Wno-misleading-indentation")
    if(BUILD_DEBUG)
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -g")
    else()
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -O3 -fopenmp")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_CXX_FLAGS}")
  endif()
endif()

####################
# Building on Windows
####################
if(WIN32)
  add_definitions(/DEIGEN_DONT_ALIGN)
  
  if(MSVC OR MINGW)
    set(MSVC_CXX_FLAGS "-std=gnu++11")
    if(BUILD_DEBUG)
      set(CMAKE_BUILD_TYPE Debug)
    else()
      set(CMAKE_BUILD_TYPE Release)
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MSVC_CXX_FLAGS}")
  endif()
endif()

#-------------------------------------------------------------------------------
# nanogui configuration and compilation
#-------------------------------------------------------------------------------
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
set(NANOGUI_USE_GLAD      ON  CACHE BOOL " " FORCE)

add_subdirectory(ext/nanogui)
include_directories(ext/nanogui/include)

set_property(TARGET glfw glfw_objects nanogui PROPERTY FOLDER "dependencies")

if(WIN32)
  set_target_properties(nanogui PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif()

# -----------------------------------------------------------------------------
# Assimp (only OBJ and FBX)
# -----------------------------------------------------------------------------
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "Disable Assimp tests" FORCE)
set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "Disable Assimp samples" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "Disable CLI tools" FORCE)
set(ASSIMP_NO_EXPORT ON CACHE BOOL "Disable all export formats" FORCE)
set(ASSIMP_BUILD_DOCS OFF CACHE BOOL "Disable Assimp docs" FORCE)

set(ASSIMP_IMPORTER_OBJ ON CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_FBX ON CACHE BOOL "" FORCE)

# Disable extra formats
set(ASSIMP_IMPORTER_3DS OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_COLLADA OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_GLTF OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_PLY OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_STL OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_X OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_BLEND OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_IFC OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_X3D OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_MMD OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_3MF OFF CACHE BOOL "" FORCE)
set(ASSIMP_IMPORTER_MS3D OFF CACHE BOOL "" FORCE)

add_subdirectory(ext/assimp)

#-------------------------------------------------------------------------------
# Find dependencies
#-------------------------------------------------------------------------------
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
if(NOT WIN32)
  find_package(Freetype REQUIRED)
endif()

#-------------------------------------------------------------------------------
# Final executable
#-------------------------------------------------------------------------------
add_executable(FinalProject
    src/main.cpp
    src/shader.cpp
    src/model.cpp
    src/animator.cpp
    src/animatedObject.cpp
    # Add more source files here as needed
)

target_include_directories(FinalProject PRIVATE
    ext/nanogui/include
    ext/nanogui/ext/eigen
    ext/nanogui/ext/nanovg/src
    ext/nanogui/ext/glfw/include
    ext/nanogui/ext/glad/include
    ext/assimp/include
)

target_link_libraries(FinalProject
    nanogui
    OpenGL::GL
    Threads::Threads
    assimp
)

# -----------------------------------------------------------------------------
# Copy shaders into build directory
# -----------------------------------------------------------------------------
file(GLOB SHADER_FILES
    "${CMAKE_SOURCE_DIR}/shaders/*.vert"
    "${CMAKE_SOURCE_DIR}/shaders/*.frag"
)

foreach(SHADER ${SHADER_FILES})
    get_filename_component(SHADER_NAME ${SHADER} NAME)
    configure_file(${SHADER} ${CMAKE_BINARY_DIR}/shaders/${SHADER_NAME} COPYONLY)
endforeach()

#-------------------------------------------------------------------------------
# Documentation
#-------------------------------------------------------------------------------
if(BUILD_DOCS)
  find_package(DOXYGEN)
  if(DOXYGEN_FOUND AND BUILD_DOCS)
    add_subdirectory(docs)
  endif()
endif()

set(CMAKE_INSTALL_PREFIX "${FinalProject_SOURCE_DIR}/")

# -----------------------------------------------------------------------------
# Unit tests
# -----------------------------------------------------------------------------
add_executable(UnitTests
    tests/test_animator.cpp
    src/animator.cpp
)

target_include_directories(UnitTests PRIVATE
    ext/nanogui/include
    ext/nanogui/ext/eigen
    ext/nanogui/ext/nanovg/src
    ext/nanogui/ext/glfw/include
    ext/nanogui/ext/glad/include
    ext/assimp/include
)

target_link_libraries(UnitTests
    nanogui
    OpenGL::GL
    Threads::Threads
    assimp
)

enable_testing()
add_test(NAME UnitTests COMMAND UnitTests)
