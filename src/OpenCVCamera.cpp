//
// Created by liuzikai on 3/11/21.
//

#include "Camera.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

namespace meta {

bool OpenCVCamera::open(const package::ParamSet &params) {
    // Start the fetching thread
    if (th) {
        close();
    }
    threadShouldExit = false;
    th = new std::thread(&OpenCVCamera::readFrameFromCamera, this, params);

    buffer[0] = cv::Mat(cv::Size(params.roi_width(), params.roi_height()), CV_8UC3);
    buffer[1] = cv::Mat(cv::Size(params.roi_width(), params.roi_height()), CV_8UC3);
    while (getFrame().empty()) std::this_thread::yield();

    return true;
}

OpenCVCamera::~OpenCVCamera() {
    if (th) {
        threadShouldExit = true;
        th->join();
        delete th;
        th = nullptr;
    }
}

void OpenCVCamera::readFrameFromCamera(const package::ParamSet &params) {
    capInfoSS.str(std::string());  // clear capInfoSS

    // Open the camera in the same thread
    // use V4L2 backend to capture video on Linux, while on Windows CAP_ANY is selected
    #ifdef BOOST_OS_WINDOWS
    cap.open(params.camera_id(), cv::CAP_ANY);
    #else
    cap.open(params.camera_id(), cv::CAP_V4L2);
    #endif
    if (!cap.isOpened()) {
        capInfoSS << "Failed to open camera " << params.camera_id() << "\n";
        std::cerr << capInfoSS.rdbuf();
        return;
    }

    // Set parameters
    if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, params.image_width())) {
        capInfoSS << "Failed to set width.\n";
    }
    if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, params.image_height())) {
        capInfoSS << "Failed to set height.\n";
    }
    capInfoSS << "Note: ROI is not effective in hardware";
    if (!cap.set(cv::CAP_PROP_FPS, params.fps())) {
        capInfoSS << "Failed to set fps.\n";
    }
    if (params.gamma().enabled()) {
        if (!cap.set(cv::CAP_PROP_GAMMA, params.gamma().val())) {
            capInfoSS << "Failed to set gamma.\n";
        }
    }
    if (params.manual_exposure().enabled()) {
        if (!cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1)) {  // manual
            capInfoSS << "Failed to set manual exposure mode.\n";
        }
        if (!cap.set(cv::CAP_PROP_EXPOSURE, params.manual_exposure().val())) {
            capInfoSS << "Failed to set exposure.\n";
        }
    } else {
        if (!cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0)) {  // auto
            capInfoSS << "Failed to set auto exposure mode.\n";
        }
    }

    // Get a test frame
    cap.read(buffer[0]);
    if (buffer->empty()) {
        capInfoSS << "Failed to fetch test image from camera " << params.camera_id() << "\n";
        std::cerr << capInfoSS.rdbuf();
        return;
    }
    if (buffer[0].cols != params.image_width() || buffer[0].rows != params.image_height()) {
        capInfoSS << "Invalid frame size. "
                  << "Expected: " << params.image_width() << "x" << params.image_height() << ", "
                  << "Actual: " << buffer[0].cols << "x" << buffer[0].rows << "\n";
        std::cerr << capInfoSS.rdbuf();
        return;
    }

    // Report actual parameters
    capInfoSS << "OpenCVCamera " << params.camera_id() << ", "
              << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT)
              << " @ " << cap.get(cv::CAP_PROP_FPS) << " fps\n"
              << "Gamma: " << cap.get(cv::CAP_PROP_GAMMA) << "\n"
              << "Auto Exposure: " << cap.get(cv::CAP_PROP_AUTO_EXPOSURE) << "\n"
              << "Exposure: " << cap.get(cv::CAP_PROP_EXPOSURE) << "\n";
    std::cout << capInfoSS.rdbuf();

    while (true) {

        uint8_t workingBuffer = 1 - lastBuffer;

        if (threadShouldExit || !cap.isOpened()) {
            bufferCaptureTime[workingBuffer] = 0;  // indicate invalid frame
            lastBuffer = workingBuffer;  // switch
            break;
        }

        if (!cap.read(buffer[workingBuffer])) {
            continue;  // try again
        }

        // Software crop
        buffer[workingBuffer] = buffer[workingBuffer](cv::Rect{
                (params.image_width() - params.roi_width()) / 2,
                (params.image_height() - params.roi_height()) / 2,
                params.roi_width(),
                params.roi_height()});

        // Save frame if required
        if (recordingVideo) {
            videoWriterMutex.lock();
            {
                if (videoWriter.isOpened()) videoWriter << buffer[workingBuffer];
            }
            videoWriterMutex.unlock();
        }

        bufferCaptureTime[workingBuffer] = (TimePoint) (cap.get(cv::CAP_PROP_POS_MSEC) * 10);

        lastBuffer = workingBuffer;

        ++cumulativeFrameCounter;  // the only place of incrementing
    }

    cap.release();
    std::cout << "OpenCVCamera: closed\n";
}

void OpenCVCamera::close() {
    if (th) {
        threadShouldExit = true;
        th->join();  // cap is managed in the thread
        delete th;
        th = nullptr;
    }
}

}