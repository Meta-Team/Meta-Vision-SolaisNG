project(TensorRTX LANGUAGES CXX CUDA)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.cu)
file(GLOB_RECURSE PLUGIN_SRCS ${PROJECT_SOURCE_DIR}/plugin/*.cu)

add_library(TensorRTX SHARED ${SRCS} ${PLUGIN_SRCS})

target_include_directories(TensorRTX INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(TensorRTX INTERFACE ${PROJECT_SOURCE_DIR}/plugin)

target_link_libraries(TensorRTX PRIVATE ${OpenCV_LIBRARIES} nvinfer cudart)

set_target_properties(TensorRTX PROPERTIES CUDA_ARCHITECTURES "86")