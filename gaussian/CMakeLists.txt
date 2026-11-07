cmake_minimum_required(VERSION 3.22)
project(my_livo_gaussian LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
if(NOT CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES native)
endif()
find_package(Torch REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc calib3d)
find_package(Eigen3 REQUIRED)
find_path(GLM_INCLUDE_DIR glm/glm.hpp REQUIRED)
add_library(gaussian_image_io STATIC image_io.cpp)
target_include_directories(gaussian_image_io PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gaussian_image_io PRIVATE ${OpenCV_LIBS})
set(GS third_party/gaussian_lic)
add_executable(my_livo_gaussian main.cpp adapter.cpp
  ${GS}/gaussian.cpp ${GS}/tinyply.cpp
  ${GS}/simple-knn/spatial.cu ${GS}/simple-knn/simple_knn.cu
  ${GS}/fused-ssim/ssim.cu
  ${GS}/rasterizer/renderer.cpp ${GS}/rasterizer/rasterizer.cpp
  ${GS}/rasterizer/rasterize_points.cu
  ${GS}/rasterizer/cuda_rasterizer/adam.cu
  ${GS}/rasterizer/cuda_rasterizer/forward.cu
  ${GS}/rasterizer/cuda_rasterizer/backward.cu
  ${GS}/rasterizer/cuda_rasterizer/rasterizer_impl.cu)
target_include_directories(my_livo_gaussian PRIVATE ${GS} ${OpenCV_INCLUDE_DIRS} ${GLM_INCLUDE_DIR})
target_link_libraries(my_livo_gaussian PRIVATE ${TORCH_LIBRARIES} CUDA::cudart Eigen3::Eigen gaussian_image_io ${OpenCV_LIBS})
separate_arguments(TORCH_FLAGS NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
target_compile_options(my_livo_gaussian PRIVATE ${TORCH_FLAGS}
  $<$<COMPILE_LANGUAGE:CXX>:-O2>
  $<$<COMPILE_LANGUAGE:CUDA>:-O2;--expt-relaxed-constexpr>)
