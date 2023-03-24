set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

add_library(TensorRTX include/postprocess.cpp include/preprocess.cu include/calibrator.cpp)
target_include_directories(TensorRTX INTERFACE include)
target_link_libraries(TensorRTX PUBLIC ${OpenCV_LIBRARIES} nvinfer cudart)
set_target_properties(TensorRTX PROPERTIES CUDA_ARCHITECTURES all-major)