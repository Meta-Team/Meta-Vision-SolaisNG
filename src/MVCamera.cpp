//
// Created by liuzikai on 7/4/21.
//

#include "Camera.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "Utilities.h"


namespace meta {

bool MVCamera::open(const package::ParamSet &params) {

    this->params = params;

    capInfoSS.str(std::string());  // clear capInfoSS

    // CameraSdkInit should be called outside

    int cameraCount = 1;
    tSdkCameraDevInfo cameraEnumList;
    tSdkCameraCapbility capability;

#define TRY_CALL(func, ...) do {                                         \
        CameraSdkStatus res = func(__VA_ARGS__);                               \
        if (res != CAMERA_STATUS_SUCCESS) {                                    \
            std::cerr << "MVCamera: " #func " returned " << res << std::endl;  \
            return false;                                                      \
        }                                                                      \
    } while(0)

    TRY_CALL(CameraEnumerateDevice, &cameraEnumList, &cameraCount);

    if (cameraCount == 0) {
        std::cerr << "CameraCoreMVCamera: no MindVision camera found" << std::endl;
        return false;
    }

    TRY_CALL(CameraInit, &cameraEnumList, -1, -1, &hCamera);
    TRY_CALL(CameraGetCapability, hCamera, &capability);

    capInfoSS << "Supported output formats: \n";
    for (int i = 0; i < capability.iMediaTypdeDesc; i++) {
        capInfoSS << "  " << capability.pMediaTypeDesc[i].acDescription << ": "
                  << capability.pMediaTypeDesc[i].iMediaType << " \n";
    }

    TRY_CALL(CameraPlay, hCamera);
    TRY_CALL(CameraSetOnceWB, hCamera);
    TRY_CALL(CameraSetIspOutFormat, hCamera, CAMERA_MEDIA_TYPE_BGR8);

    capInfoSS << "Note: FPS is not effective.\n";

    if (params.manual_exposure().enabled()) {
        if (CameraSetAeState(hCamera, FALSE) != CAMERA_STATUS_SUCCESS) {  // manual
            capInfoSS << "Failed to set manual exposure mode.\n";
        }
        if (CameraSetExposureTime(hCamera, params.manual_exposure().val()) != CAMERA_STATUS_SUCCESS) {
            capInfoSS << "Failed to set exposure.\n";
        }
    } else {
        if (CameraSetAeState(hCamera, TRUE) != CAMERA_STATUS_SUCCESS) {  // auto
            capInfoSS << "Failed to set auto exposure mode.\n";
        }
    }

    tSdkImageResolution resolution = capability.pImageSizeDesc[0];
    resolution.iIndex = 0xFF;  // customized ROI
    resolution.iHOffsetFOV = (params.image_width() - params.roi_width()) / 2;
    resolution.iVOffsetFOV = (params.image_height() - params.roi_height()) / 2;
    resolution.iWidthFOV = resolution.iWidth = params.roi_width();
    resolution.iHeightFOV = resolution.iHeight = params.roi_height();
    TRY_CALL(CameraSetImageResolution, hCamera, &resolution);
    capInfoSS << "Note: ROI enabled.\n";

    // Setup callback
    buffer[0] = cv::Mat(cv::Size(params.roi_width(), params.roi_height()), CV_8UC3);
    buffer[1] = cv::Mat(cv::Size(params.roi_width(), params.roi_height()), CV_8UC3);
    shouldFetchNextFrame = true;
    TRY_CALL(CameraSetCallbackFunction, hCamera, &MVCamera::newFrameCallback, this, nullptr);

    // Wait for a test frame
    while (shouldFetchNextFrame) std::this_thread::yield();
    cv::Mat testFrame = getFrame();
    fetchNextFrame();

    if (buffer->empty()) {
        capInfoSS << "Failed to fetch test image from camera " << params.camera_id() << "\n";
        std::cerr << capInfoSS.rdbuf();
        return false;
    }
    if (buffer[0].cols != params.roi_width() || buffer[0].rows != params.roi_height()) {
        capInfoSS << "Invalid frame size. "
                  << "Expected: " << params.roi_width() << "x" << params.roi_height() << ", "
                  << "Actual: " << buffer[0].cols << "x" << buffer[0].rows << "\n";
        std::cerr << capInfoSS.rdbuf();
        return false;
    }

    // Report actual parameters
    int realGamma, realAutoExposure;
    double realExposureTime;

    TRY_CALL(CameraGetGamma, hCamera, &realGamma);
    TRY_CALL(CameraGetAeState, hCamera, &realAutoExposure);
    realAutoExposure = (realAutoExposure == TRUE ? 0 : 1);  // 0 = auto, 1 = manual
    TRY_CALL(CameraGetExposureTime, hCamera, &realExposureTime);

    capInfoSS << "MVCVCamera " << params.camera_id() << ", "
              << params.roi_width() << "x" << params.roi_height()
              << " @ " << params.fps() << " fps\n"
              << "Gamma: " << realGamma << "\n"
              << "Auto Exposure: " << realAutoExposure << "\n"
              << "Exposure: " << realExposureTime << "\n";
    std::cout << capInfoSS.rdbuf();

    return true;

#undef TRY_CALL
}

void MVCamera::newFrameCallback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead *pFrameHead, PVOID pContext) {

    auto p = static_cast<MVCamera *>(pContext);

    p->cumulativeFrameCounter++;  // count frame even if it's not processed

    if (!p->shouldFetchNextFrame) {  // do not process if not required

        CameraReleaseImageBuffer(hCamera, pFrameBuffer);
        return;

    } else {

        tSdkFrameHead frameInfo = *pFrameHead;  // make a copy

        uint8_t workingBuffer = 1 - p->lastBuffer;

        cv::Mat &image = p->buffer[workingBuffer];
        if (image.cols != p->params.roi_width() || p->params.roi_height()) {
            image = cv::Mat(cv::Size(p->params.roi_width(), p->params.roi_height()), CV_8UC3);
        }

        auto res = CameraImageProcess(hCamera, pFrameBuffer, image.data, &frameInfo);  // load directly into buffer
        if (res != CAMERA_STATUS_SUCCESS) {
            std::cerr << "MVCamera: CameraImageProcess returned " << res << std::endl;
            return;
        }

        if (p->recordingVideo) {
            p->videoWriterMutex.lock();
            {
                if (p->videoWriter.isOpened()) p->videoWriter << image;
            }
            p->videoWriterMutex.unlock();
        }

        p->bufferCaptureTime[workingBuffer] = frameInfo.uiTimeStamp;

        // Switch frame
        p->lastBuffer = workingBuffer;
        p->shouldFetchNextFrame = false;

        CameraReleaseImageBuffer(hCamera, pFrameBuffer);
        return;
    }

}

void MVCamera::close() {
    CameraUnInit(hCamera);
    bufferCaptureTime[0] = bufferCaptureTime[1] = 0;  // indicate invalid frame
    hCamera = 0;
}

MVCamera::~MVCamera() {
    if (hCamera) CameraUnInit(hCamera);
}

void MVCamera::fetchNextFrame() {
    shouldFetchNextFrame = true;
}

}