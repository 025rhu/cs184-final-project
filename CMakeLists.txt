cmake_minimum_required(VERSION 3.14)
project(final_project)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Optional: Build type toggle
option(BUILD_DEBUG "Build with debug settings" OFF)

if (BUILD_DEBUG)
    set(CMAKE_BUILD_TYPE Debug)
else()
    set(CMAKE_BUILD_TYPE Release)
endif()

# -----------------------------------------------------------------------------
# Platform-specific settings
# -----------------------------------------------------------------------------
if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -m64")
elseif(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -m64 -Wno-deprecated-declarations")
elseif(WIN32)
    add_definitions(/DEIGEN_DONT_ALIGN)  # For Eigen (used in some projects)
endif()

# -----------------------------------------------------------------------------
# Required system packages
# -----------------------------------------------------------------------------
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

# -----------------------------------------------------------------------------
# FetchContent (for header-only libs like Eigen)
# -----------------------------------------------------------------------------
include(FetchContent)

# Eigen
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

# -----------------------------------------------------------------------------
# NanoGUI (includes GLFW and GLAD)
# -----------------------------------------------------------------------------
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
set(NANOGUI_USE_GLAD      ON  CACHE BOOL " " FORCE)

add_subdirectory(ext/nanogui)
include_directories(ext/nanogui/include)

# -----------------------------------------------------------------------------
# Assimp
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# Assimp (only enable OBJ and FBX support)
# -----------------------------------------------------------------------------
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "Disable Assimp tests" FORCE)
set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "Disable Assimp samples" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "Disable Assimp CLI tools" FORCE)
set(ASSIMP_NO_EXPORT ON CACHE BOOL "Disable all export formats" FORCE)
set(ASSIMP_BUILD_DOCS OFF CACHE BOOL "Disable Assimp docs" FORCE)

# Only enable OBJ and FBX importers
set(ASSIMP_IMPORTER_OBJ ON CACHE BOOL "Enable OBJ importer" FORCE)
set(ASSIMP_IMPORTER_FBX ON CACHE BOOL "Enable FBX importer" FORCE)

# Disable all other importers
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
# Add more OFF lines here if you see other importers being compiled

add_subdirectory(ext/assimp)


# -----------------------------------------------------------------------------
# Executable and source files
# -----------------------------------------------------------------------------
add_executable(final_project
    main.cpp
    # Add other source files here
)

# -----------------------------------------------------------------------------
# Link libraries
# -----------------------------------------------------------------------------
target_link_libraries(final_project
    PRIVATE
        assimp
        nanogui
        glfw
        OpenGL::GL
        Threads::Threads
)

# -----------------------------------------------------------------------------
# Include additional headers
# -----------------------------------------------------------------------------
target_include_directories(final_project
    PRIVATE
        ext/nanogui/include
        ext/assimp/include
        ${eigen_SOURCE_DIR}     # <-- Adds Eigen headers
)
