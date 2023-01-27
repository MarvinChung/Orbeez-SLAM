cmake_minimum_required(VERSION 3.19)
project(ORBEEZ LANGUAGES CXX CUDA)

option(ORBEEZ_BUILD_WITH_GUI "Build with GUI support (requires GLFW and GLEW)?" ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if( ORBEEZ_BUILD_WITH_GUI )
    set(NGP_BUILD_WITH_GUI ON CACHE BOOL "build ngp gui" FORCE)
    list(APPEND ORBEEZ_DEFINITIONS -DORBEEZ_GUI)
else ()
    set(NGP_BUILD_WITH_GUI OFF CACHE BOOL "not building ngp gui" FORCE)
endif( ORBEEZ_BUILD_WITH_GUI )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# ##############################################################################
# Build type and C++ compiler setup
# ##############################################################################

# Set a default configuration if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "No release type specified. Setting to 'Release'.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo")
endif()

if(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_WARNINGS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP24")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

# ##############################################################################
# CUDA compiler setup
# ##############################################################################
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)
set(CUDA_LINK_LIBRARIES_KEYWORD PUBLIC)

get_directory_property(ORBEEZ_HAS_PARENT PARENT_DIRECTORY)

if(MSVC)
    list(APPEND CUDA_NVCC_FLAGS "-Xcompiler=-bigobj")
else()
    list(APPEND CUDA_NVCC_FLAGS "-Xcompiler=-mf16c")
    list(APPEND CUDA_NVCC_FLAGS "-Xcompiler=-Wno-float-conversion")
    list(APPEND CUDA_NVCC_FLAGS "-Xcompiler=-fno-strict-aliasing")
    list(APPEND CUDA_NVCC_FLAGS "-Xcompiler=-fPIC")
    set(CUDA_TOOLKIT_ROOT_DIR /opt/cuda/targets/x86_64-linux)
endif()

list(APPEND CUDA_NVCC_FLAGS "--extended-lambda")
list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")

# ##############################################################################
# Dependencies (ordered alphabetically)
# ##############################################################################

# ###############
# DBoW2 #
# ###############
add_subdirectory(Thirdparty/DBoW2)
list(APPEND IncludeList "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/DBoW2")

# ###############
# g2o #
# ###############
set(G2O_BUILD_APPS OFF CACHE BOOL "build g2o apps" FORCE)
set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "build g2o examples" FORCE)
set(BUILD_WITH_MARCH_NATIVE ON CACHE BOOL "build with -march=native" FORCE)
add_subdirectory(Thirdparty/g2o)

list(APPEND IncludeList "${PROJECT_BINARY_DIR}/Thirdparty/g2o")
list(APPEND IncludeList "Thirdparty/g2o")

# list(APPEND IncludeList "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/ceres-solver/include")
set(G2O_LIBRARIES
    core
    freeglut_minimal
    opengl_helper
    solver_dense
    solver_eigen
    solver_pcg
    solver_slam2d_linear
    solver_structure_only
    stuff
    types_data
    types_icp
    types_sba
    types_sclam2d
    types_sim3
    types_slam2d_addons
    types_slam2d
    types_slam3d_addons
    types_slam3d
)

# ###############
# instant-ngp #
# ###############
add_subdirectory(Thirdparty/instant-ngp-kf)

list(APPEND IncludeList "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/instant-ngp-kf/include")
list(APPEND IncludeList "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/instant-ngp-kf/src")

# ##############################################################################
# Packages
# ##############################################################################
find_package(OpenCV 4 PATHS "/usr/include" "${CMAKE_SOURCE_DIR}/Thirdparty/opencv-4.5.5/build")

if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)

    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ##############################################################################
# Program
# ##############################################################################
list(APPEND IncludeList "${CMAKE_CURRENT_SOURCE_DIR}/include")
list(APPEND IncludeList "${EIGEN3_INCLUDE_DIR}")
list(APPEND IncludeList "${PROJECT_SOURCE_DIR}")

message(VERBOSE "g2o_SOURCE_DIR: " ${g2o_SOURCE_DIR})
message(VERBOSE "G2O_LIBRARIES: " ${G2O_LIBRARIES})
message(VERBOSE "g2o_LIBRARY_OUTPUT_DIRECTORY: " ${g2o_LIBRARY_OUTPUT_DIRECTORY})

set(SOURCES
    src/Converter.cu
    src/Frame.cu
    src/FrameDrawer.cu
    src/Initializer.cu
    src/KeyFrame.cu
    src/KeyFrameDatabase.cu
    src/Map.cu
    src/MapDrawer.cu
    src/MapPoint.cu
    src/NerfMapping.cu
    src/Optimizer.cu
    src/ORBextractor.cu
    src/ORBmatcher.cu
    src/PnPsolver.cu
    src/Sim3Solver.cu
    src/System.cu
    src/Tracking.cu
    src/Viewer.cu
    src/LoopClosing.cu
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR})

get_filename_component(CUDA_COMPILER_BIN "${CMAKE_CUDA_COMPILER}" DIRECTORY)

set(ORBEEZ_LIBRARIES

    # ${NGP_LIBRARIES}
    # ${EIGEN3_LIBS}
    ${G2O_LIBRARIES}

    # ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${OpenCV_LIBS}

    # ${NGP_LIBRARIES}
    ngp

    # tiny-cuda-nn
    # CUDA::cudart
    DBoW2

    # glog
    # X11
    # Threads::Threads
)

if(ORBEEZ_HAS_PARENT)
    set(CMAKE_CUDA_ARCHITECTURES ${CMAKE_CUDA_ARCHITECTURES} PARENT_SCOPE)
endif()

if(ORBEEZ_HAS_PARENT)
    set(ORBEEZ_LIBRARIES ${ORBEEZ_LIBRARIES} PARENT_SCOPE)
endif()

add_library(ORBEEZ STATIC ${SOURCES})

target_link_libraries(ORBEEZ PUBLIC
    ${G2O_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ngp
	DBoW2
)

target_include_directories(ORBEEZ PUBLIC
    ${IncludeList}
    ${PROJECT_BINARY_DIR}
    ${PROJECT_BINARY_DIR}/Thirdparty/instant-ngp-kf/
    build/Thirdparty/instant-ngp-kf/
)


set_target_properties(ORBEEZ PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set_target_properties(ORBEEZ PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_compile_options(ORBEEZ PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(ORBEEZ PUBLIC ${ORBEEZ_DEFINITIONS})

if(Python_FOUND)
    add_library(pyngp SHARED ThirdParty/instant-ngp-kf/src/python_api.cu)
    set_target_properties(pyngp PROPERTIES CXX_VISIBILITY_PRESET "hidden")
    set_target_properties(pyngp PROPERTIES CUDA_VISIBILITY_PRESET "hidden")
    target_link_libraries(pyngp PUBLIC ngp ${PYTHON_LIBRARIES} pybind11::module)
    target_compile_options(pyngp PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
    target_compile_definitions(pyngp PUBLIC -DNGP_PYTHON ${NGP_DEFINITIONS} ${TCNN_DEFINITIONS})
    pybind11_extension(pyngp)

    if(MSVC)
        # Copy dlls needed at runtime
        file(GLOB CUBLAS_DLLS "${CUDA_COMPILER_BIN}/cublas*.dll")

        if(CUBLAS_DLLS)
            add_custom_command(TARGET pyngp POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy ${CUBLAS_DLLS} $<TARGET_FILE_DIR:pyngp>
                COMMAND_EXPAND_LISTS
            )
        endif()
    endif()
endif()

message(STATUS "ORBEEZ_BUILD_WITH_GUI:" ${ORBEEZ_BUILD_WITH_GUI})
message(STATUS "ORBEEZ_DEFINITIONS:" ${ORBEEZ_DEFINITIONS})

# Build examples

# Monocular
add_executable(mono_tum Examples/Monocular/mono_tum.cu)
target_link_libraries(mono_tum ORBEEZ ngp tiny-cuda-nn)
target_compile_options(mono_tum PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(mono_tum PUBLIC ${ORBEEZ_DEFINITIONS})

add_executable(mono_replica Examples/Monocular/mono_replica.cu)
target_link_libraries(mono_replica ORBEEZ ngp tiny-cuda-nn)
target_compile_options(mono_replica PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(mono_replica PUBLIC ${ORBEEZ_DEFINITIONS})

add_executable(mono_scannet Examples/Monocular/mono_scannet.cu)
target_link_libraries(mono_scannet ORBEEZ ngp tiny-cuda-nn)
target_compile_options(mono_scannet PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(mono_scannet PUBLIC ${ORBEEZ_DEFINITIONS})

# RGBD
add_executable(rgbd_tum Examples/RGB-D/rgbd_tum.cu)
target_link_libraries(rgbd_tum ORBEEZ ngp tiny-cuda-nn)
target_compile_options(rgbd_tum PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(rgbd_tum PUBLIC ${ORBEEZ_DEFINITIONS})

add_executable(rgbd_replica Examples/RGB-D/rgbd_replica.cu)
target_link_libraries(rgbd_replica ORBEEZ ngp tiny-cuda-nn)
target_compile_options(rgbd_replica PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(rgbd_replica PUBLIC ${ORBEEZ_DEFINITIONS})

add_executable(rgbd_scannet Examples/RGB-D/rgbd_scannet.cu)
target_link_libraries(rgbd_scannet ORBEEZ ngp tiny-cuda-nn)
target_compile_options(rgbd_scannet PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(rgbd_scannet PUBLIC ${ORBEEZ_DEFINITIONS})
