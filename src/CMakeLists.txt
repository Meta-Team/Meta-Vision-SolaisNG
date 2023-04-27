# Parameters and results ProtoBuf
if (Protobuf_FOUND)
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS Parameters.proto)
    add_library(libParameters
            ${PROTO_SRCS} ${PROTO_HDRS})
    target_link_libraries(libParameters
            PUBLIC ${Protobuf_LIBRARIES})
    target_include_directories(libParameters
            PUBLIC ${Protobuf_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}  # for Parameters.pc.h
            )
else()
    message("=> Target libParameters is not available to build. Depends: Protobuf")
endif ()

# libTerminalSocket
if (Boost_FOUND AND spdlog_FOUND)
    add_library(libTerminalSocket TerminalSocket.cpp)
    target_link_libraries(libTerminalSocket PUBLIC
            ${mingw_winsock} ${mingw_mswsock}  ${Boost_SYSTEM_LIBRARY} spdlog::spdlog
            )
else()
    message("=> Target libTerminalSocket is not available to build. Depends: Boost")
endif ()

# libSerial
if (Boost_FOUND)
    if(DEFINED SERIAL_DEVICE)
        message("Serial device is set as \"${SERIAL_DEVICE}\"")
    else()
        if(MINGW)
        # If you use CH340 series converter on Windows(MINGW)
            set(SERIAL_DEVICE "COM4")
        else()
        # If you use tegra high speed UART on Jetson Nano kit
        #   set(SERIAL_DEVICE "/dev/ttyTHS1")
        # If you use CH340 series converter on Linux
            set(SERIAL_DEVICE "/dev/ttyUSB0")
        endif()
        message("Use default serial device \"${SERIAL_DEVICE}\"")
    endif ()

    add_library(libSerial Serial.cpp CRC.cpp)
    target_compile_definitions(libSerial
            PUBLIC "SERIAL_DEVICE=\"${SERIAL_DEVICE}\"")
    target_link_libraries(libSerial PUBLIC ${Boost_SYSTEM_LIBRARY})
else()
    message("=> Target libSerial is not available to build. Depends: Boost")
endif ()

if (OpenCV_FOUND AND CUDAToolkit_FOUND AND spdlog_FOUND)
    add_library(libYOLODet YOLOv5_TensorRT.cpp)
    target_link_libraries(libYOLODet ${OpenCV_LIBRARIES} nvinfer cudart fmt::fmt nvonnxparser ${CUDA_CUDART} ${CUDA_cuda_driver_LIBRARY} spdlog::spdlog)
else()
    message("=> Target libYOLODet is not available to build. Depends: OpenCV")
endif()

# libArmorSolver
if (OpenCV_FOUND)
    add_library(libArmorSolver PositionCalculator.cpp)
    target_link_libraries(libArmorSolver ${OpenCV_LIBRARIES})
else()
    message("=> Target libArmorSolver is not available to build. Depends: OpenCV")
endif()

# libCamera
if (OpenCV_FOUND AND TARGET libParameters)
    add_library(libCamera Camera.cpp OpenCVCamera.cpp MVCamera.cpp)
    target_link_libraries(libCamera ${OpenCV_LIBRARIES} libParameters MVSDK)
    if (GSTREAMER_FOUND)
        target_compile_definitions(libParameters PUBLIC "GSTREAMER_FOUND=1")
        message("GStreamer found")
    else()
        message("GStreamer not found")
    endif()
else()
    message("=> Target libCamera is not available to build. Depends: OpenCV, libParameters, MVSDK")
endif()

# libGStreamerCamera
#if (OpenCV_FOUND AND TARGET libParameters)
#    add_library(libGStreamerCamera GStreamerCamera.cpp)
#    target_link_libraries(libGStreamerCamera
#            ${GSTREAMER_LIBRARIES}
#            ${GSTREAMER-APP_LIBRARIES}
#            ${GSTREAMER-AUDIO_LIBRARIES}
#            ${GSTREAMER-PBUTILS_LIBRARIES}
#            ${GSTREAMER-FFT_LIBRARIES}
#            ${OpenCV_LIBRARIES}
#            libParameters
#            )
#else()
#    message("=> Target libGStreamerCamera is not available to build. Depends: GStreamer, OpenCV, libParameters")
#endif ()

# libSolais
if (Boost_FOUND AND OpenCV_FOUND AND Protobuf_FOUND AND TARGET libParameters AND TARGET libTerminalSocket AND TARGET libArmorSolver AND TARGET libSerial)
    add_library(libSolais
            ArmorDetector.cpp
            AimingSolver.cpp
            ParamSetManager.cpp
            ImageSet.cpp
            VideoSet.cpp
            Executor.cpp)
    if(TARGET libYOLODet)
        list(INSERT SolaisNNLibs 0 
            libYOLODet
        )
    endif()
    target_link_libraries(libSolais
            ${OpenCV_LIBRARIES}
            ${Boost_FILESYSTEM_LIBRARY}
            ${Boost_SYSTEM_LIBRARY}
            ${Protobuf_LIBRARIES}
            pthread
            libParameters
            libTerminalSocket
            libSerial
            libArmorSolver
            libCamera
            ${SolaisNNLibs}
            )
    target_compile_definitions(libSolais PUBLIC "$<$<CONFIG:DEBUG>:DEBUG>")

    if(NOT PARAM_SET_ROOT)
        set(PARAM_SET_ROOT "${PROJECT_SOURCE_DIR}/data")
        message("Directory of parameter sets is default as \"${PARAM_SET_ROOT}\"")
    else()
        message("Directory of parameter sets is set as \"${PARAM_SET_ROOT}\"")
    endif ()

    if(NOT DATA_SET_ROOT)
        set(DATA_SET_ROOT "${PROJECT_SOURCE_DIR}/data")
        message("Directory of data sets is default as \"${DATA_SET_ROOT}\"")
    else()
        message("Directory of data sets is set as \"${DATA_SET_ROOT}\"")
    endif ()

    target_compile_definitions(libSolais
            PUBLIC "PARAM_SET_ROOT=\"${PARAM_SET_ROOT}\""
            PUBLIC "DATA_SET_ROOT=\"${DATA_SET_ROOT}\"")
else()
    message("=> Target libSolais is not available to build. Depends: Boost, OpenCV, Protobuf, fmt, spdlog, libParameters, libTerminalSocket, libSerial, libArmorSolver")
endif()

## libNanoDet
#if (OpenCV_FOUND AND CUDAToolkit_FOUND)
#    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
#    add_library(libNanoDet SHARED NanoDet_TensorRT.cu)
#    target_link_libraries(libNanoDet ${OpenCV_LIBRARIES} nvinfer cudart)
#    target_include_directories(libNanoDet INTERFACE include)
#else()
#    message("=> Target libNanoDet is not available to build. Depends: OpenCV")
#endif()
