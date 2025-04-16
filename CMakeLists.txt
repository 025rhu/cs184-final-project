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
option(BUILD_LIBCGL    "Build with libCGL"            ON)
option(BUILD_DEBUG     "Build with debug settings"    OFF)
option(BUILD_DOCS      "Build documentation"          OFF)

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
  if(NOT BUILD_LIBCGL)
    include_directories( "/System/Library/Frameworks" )
    find_library (COCOA_LIBRARIES Cocoa)
    find_library (IOKIT_LIBRARIES IOkit)
    find_library (COREVIDEO_LIBRARIES CoreVideo)
  endif()

  # Clang configuration
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

    set(CLANG_CXX_FLAGS "-std=c++11 -m64")

    if(BUILD_DEBUG)
      set(CMAKE_BUILD_TYPE Debug)
      set(CLANG_CXX_FLAGS "${CLANG_CXX_FLAGS} -g")
    else(BUILD_DEBUG)
      set(CLANG_CXX_FLAGS "${CLANG_CXX_FLAGS} -O3")
      set(CLANG_CXX_FLAGS "${CLANG_CXX_FLAGS} -funroll-loops")
      set(CLANG_CXX_FLAGS "${CLANG_CXX_FLAGS} -Wno-narrowing")
      set(CLANG_CXX_FLAGS "${CLANG_CXX_FLAGS} -Wno-deprecated-register")
    endif(BUILD_DEBUG)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CLANG_CXX_FLAGS}")

  endif()

  # GCC configuration
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

    set(GCC_CXX_FLAGS "-std=gnu++11 -m64")

    if(BUILD_DEBUG)
      set(CMAKE_BUILD_TYPE Debug)
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -g")
    else(BUILD_DEBUG)
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -O3")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -fopenmp")
    endif(BUILD_DEBUG)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_CXX_FLAGS}")

  endif()

endif(APPLE)

##################
# Build on Linux #
##################
if(UNIX AND NOT APPLE)

  # GCC only
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

    set(GCC_CXX_FLAGS "-std=gnu++11 -m64")
    set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -Wno-deprecated-declarations")
    set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -Wno-misleading-indentation")

    # X11 Dependencies
    if(NOT BUILD_LIBCGL)
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXi")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXxf86vm")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXinerama")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXcursor")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXfixes")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXrandr")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXext")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXrender")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lX11")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lpthread")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lxcb")
      set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -lXau")
    endif()

    # Debug configuration
    if(BUILD_DEBUG)
        set(CMAKE_BUILD_TYPE Debug)
        set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -g")
    else(BUILD_DEBUG)
        set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -O3")
        set(GCC_CXX_FLAGS "${GCC_CXX_FLAGS} -fopenmp")
    endif(BUILD_DEBUG)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_CXX_FLAGS}")

  endif()

endif()

####################
# Build on Windows #
####################
if(WIN32)

  # EIGEN fix:
  # See "Cause 4" here:
  # https://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html

  # NOTE: future maintainers, you may want to use add_compile_definitions instead.
  # this was added because too many students are using old CMake (<3.12) versions.
  add_definitions(/DEIGEN_DONT_ALIGN)

  if(MSVC)

    set(MSVC_CXX_FLAGS "-std=gnu++11")

    if(BUILD_DEBUG)
        set(CMAKE_BUILD_TYPE Debug)
    else(BUILD_DEBUG)
        set(CMAKE_BUILD_TYPE Release)
    endif(BUILD_DEBUG)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MSVC_CXX_FLAGS}")
    
  endif(MSVC)

  if(MINGW)

    set(MSVC_CXX_FLAGS "-std=gnu++11")

    if(BUILD_DEBUG)
        set(CMAKE_BUILD_TYPE Debug)
    else(BUILD_DEBUG)
        set(CMAKE_BUILD_TYPE Release)
    endif(BUILD_DEBUG)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MSVC_CXX_FLAGS}")

  endif(MINGW)

endif(WIN32)

#-------------------------------------------------------------------------------
# nanogui configuration and compilation
#-------------------------------------------------------------------------------

# Disable building extras we won't need (pure C++ project)
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
set(NANOGUI_USE_GLAD      ON  CACHE BOOL " " FORCE)

# Add the configurations from nanogui
add_subdirectory(ext/nanogui)
include_directories(ext/nanogui/include)

# For reliability of parallel build, make the NanoGUI targets dependencies
set_property(TARGET glfw glfw_objects nanogui PROPERTY FOLDER "dependencies")

# For Windows, set the library output directory to put the DLL's next
# to the binary. I tried to use add_custom_command to just do a copy as a
# POST_BUILD setting, but for some reason no matter what the command does,
# Visual Studio will complain about its solution file being modified?
# In the interest of avoiding the flood of Piazza posts inquiring about this,
# we take the more robust route.
if(WIN32)
  # Also worth mentioning is that since NANOGUI produces a DLL on windows,
  # it is considered a "RUNTIME" and not a "LIBRARY" target according to CMake.
  # See https://cmake.org/cmake/help/v3.0/prop_tgt/RUNTIME_OUTPUT_DIRECTORY.html
  # > For DLL platforms the DLL part of a shared library is treated as a runtime target
  set_target_properties(nanogui PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif(WIN32)


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

# Required packages
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
if(NOT WIN32)
  find_package(Freetype REQUIRED)
endif()

# CGL
if(BUILD_LIBCGL)
  add_subdirectory(ext/CGL)
  include_directories(ext/CGL/include)
else(BUILD_LIBCGL)
  find_package(CGL REQUIRED)
endif(BUILD_LIBCGL)

#-------------------------------------------------------------------------------
# Add subdirectories
#-------------------------------------------------------------------------------
# add_subdirectory(src)


# -----------------------------------------------------------------------------
# Final executable
# -----------------------------------------------------------------------------
add_executable(FinalProject
    src/main.cpp
    src/shader.cpp
    src/model.cpp
    src/animator.cpp
    # Add more source files here as needed
)

target_include_directories(FinalProject PRIVATE
    ext/nanogui/include
    ext/nanogui/ext/eigen
    ext/nanogui/ext/nanovg/src
    ext/nanogui/ext/glfw/include
    ext/nanogui/ext/glad/include
    ext/assimp/include
    ext/CGL/include
)

target_link_libraries(FinalProject
    nanogui
    # glfw
    OpenGL::GL
    Threads::Threads
    assimp
    CGL
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



# build documentation
if(BUILD_DOCS)
  find_package(DOXYGEN)
  if(DOXYGEN_FOUND AND BUILD_DOCS)
    add_subdirectory(docs)
  endif()
endif()

# Install settings
set(CMAKE_INSTALL_PREFIX "${FinalProject_SOURCE_DIR}/")
