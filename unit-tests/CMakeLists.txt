# TerminalSocketUnitTest
if (TARGET libTerminalSocket)
    add_executable(TerminalSocketUnitTest TerminalSocketUnitTest.cpp)
    target_link_libraries(TerminalSocketUnitTest libTerminalSocket)
else ()
    message("=> Target TerminalSocketUnitTest is not available to build. Depends: libTerminalSocket")
endif ()

# SerialUnitTest
if (TARGET libSerial)
    add_executable(SerialUnitTest SerialUnitTest.cpp)
    target_link_libraries(SerialUnitTest libSerial pthread)
else ()
    message("=> Target SerialUnitTest is not available to build. Depends: libSerial")
endif ()

# CameraBenchmark
if (TARGET libCamera)
    add_executable(CameraBenchmark CameraBenchmark.cpp)
    target_link_libraries(CameraBenchmark libCamera)
else ()
    message("=> Target CameraBenchmark is not available to build. Depends: libCamera")
endif ()

# PositionCalculatorUnitTest
find_package(PkgConfig REQUIRED)
if (PkgConfig_FOUND)
    pkg_check_modules(ZBar IMPORTED_TARGET zbar)
endif ()
if (ZBar_FOUND AND TARGET libArmorSolver)
    add_executable(PositionCalculatorUnitTest PositionCalculatorUnitTest.cpp)
    target_link_libraries(PositionCalculatorUnitTest libArmorSolver PkgConfig::ZBar)
else ()
    message("=> Target PositionCalculatorUnitTest is not available to build. Depends: ZBar, libArmorSolver")
endif ()

# GStreamerUnitTest
if (GSTREAMER_FOUND)
    add_executable(GStreamerUnitTest GStreamerUnitTest.cpp)
    target_link_libraries(GStreamerUnitTest
            ${GSTREAMER_LIBRARIES}
            ${GSTREAMER_APP_LIBRARIES}
            ${GSTREAMER_AUDIO_LIBRARIES}
            ${GSTREAMER_PBUTILS_LIBRARIES}
            ${GSTREAMER_FFT_LIBRARIES}
            ${OpenCV_LIBRARIES}
            )
else ()
    message("=> Target GStreamerUnitTest is not available to build. Depends: GStreamer")
endif ()

# MVCameraOpenCVDemo
if (OpenCV_FOUND)
    add_executable(MVCameraOpenCVDemo MVCameraOpenCVDemo.cpp)
    target_link_libraries(MVCameraOpenCVDemo
            ${OpenCV_LIBRARIES}
            MVSDK
            )
else ()
    message("=> Target MVCameraOpenCVDemo is not available to build. Depends: OpenCV, MVSDK")
endif ()

# TensorRTUnitTest
if (OpenCV_FOUND AND CUDAToolkit_FOUND)
    add_executable(TensorRTUnitTest TensorRTX_YOLOv5UnitTest.cpp)
    target_link_libraries(TensorRTUnitTest
            ${OpenCV_LIBRARIES}
            nvinfer cudart
            TensorRTX
            )
    add_executable(TensorRT_NanoDet_UnitTest TensorRT_NanoDet_UnitTest.cpp)
    target_link_libraries(TensorRT_NanoDet_UnitTest
            ${OpenCV_LIBRARIES}
            nvinfer cudart
            libNanoDet
            )
    add_executable(CudaTest CudaTest.cu)
    target_link_libraries(CudaTest
            ${OpenCV_LIBRARIES}
            nvinfer cudart
            )
else ()
    message("=> Target TensorRTUnitTest is not available to build. Depends: OpenCV, TensorRT, CUDAToolkit")
endif ()

