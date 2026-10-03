cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/Modules/")

project(FluidSimulation)

add_executable(FluidSimulation
    src/Fluid.h
    src/Simulation.cpp
    src/FluidVisualizer.cpp
    src/FluidVisualizer.h
    src/Grid2.h
    src/main.cpp
    src/Scene.cpp
    src/Scene.h
    src/Containers/Array2.h
    src/Containers/Index2.cpp
    src/Containers/Index2.h
    src/Math/AABox2.h
    src/Math/Vector2.cpp
    src/Math/Vector2.h
    src/Math/Vector3.cpp
    src/Math/Vector3.h
    src/Numeric/SparseMatrix.h
    src/Numeric/PCGSolver.h
    src/Std/CMath.h
    src/Std/CStdInt.h
    )

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

target_include_directories(FluidSimulation
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${OPENGL_INCLUDE_DIR}
    ${GLUT_INCLUDE_DIR}
    )

target_link_libraries(FluidSimulation
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    )

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT FluidSimulation)

if (WIN32)
    add_custom_command(
        TARGET FluidSimulation
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different
        "${PROJECT_SOURCE_DIR}/External/freeglut-3.2.1/build-vs2017/bin/$<CONFIG>/freeglut$<$<CONFIG:Debug>:d>.dll"
        "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>"
        )
endif ()
