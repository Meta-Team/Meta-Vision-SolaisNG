add_subdirectory(mvsdk)
if(CUDAToolkit_FOUND)
add_subdirectory(TensorRTX_YOLOv5)
endif()
