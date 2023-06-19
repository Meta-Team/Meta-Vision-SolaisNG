if(OpenCV_FOUND)
    # message("=> Targets OpenCV utilities are available to build")

    add_executable(CameraCalibration CameraCalibration.cpp)
    target_link_libraries(CameraCalibration ${OpenCV_LIBRARIES})

    add_executable(ExtractFrameFromVideo ExtractFrameFromVideo.cpp)
    target_link_libraries(ExtractFrameFromVideo ${OpenCV_LIBRARIES})

    add_executable(TakePhotoUsingCamera TakePhotoUsingCamera.cpp)
    target_link_libraries(TakePhotoUsingCamera ${OpenCV_LIBRARIES})
else()
    message("=> Targets OpenCV utilities is not available to build. Depends: OpenCV")
endif()
