//
// Created by liuzikai on 6/27/21.
//

#include "VideoSet.h"
#include "Utilities.h"
#include <iostream>
#include <iomanip>
#include <opencv2/imgproc/imgproc.hpp>

namespace meta {

// DATA_SET_ROOT defined in CMakeLists.txt
VideoSet::VideoSet() : videoSetRoot(fs::path(DATA_SET_ROOT) / "videos") {
    if (!fs::exists(videoSetRoot)) {
        fs::create_directories(videoSetRoot);
    }
}

void VideoSet::reloadVideoList() {
    videos.clear();
    if (fs::is_directory(videoSetRoot)) {
        for (const auto &entry : fs::directory_iterator(videoSetRoot)) {
#ifdef BOOST_OS_WINDOWS
            if (wcscmp(entry.path().extension().c_str(), L".mkv") == 0 ||
                wcscmp(entry.path().extension().c_str(), L".avi") == 0) {
#else
            if (strcasecmp(entry.path().extension().c_str(), ".mkv") == 0 ||
                strcasecmp(entry.path().extension().c_str(), ".avi") == 0) {
#endif
                videos.emplace_back(entry.path().filename().string());
            }
        }
    }

}

cv::Mat VideoSet::getVideoFirstFrame(const std::string &videoName, const ParamSet &params) const {
    cv::VideoCapture video((videoSetRoot / videoName).string());
    if (!video.isOpened()) return cv::Mat();
    cv::Mat img;
    video >> img;
    if (!img.empty() && img.rows != params.roi_height() || img.cols != params.roi_width()) {
        cv::resize(img, img, cv::Size(params.roi_width(), params.roi_height()));
    }
    return img;
}

bool VideoSet::openVideo(const std::string &videoName, const ParamSet &params) {
    if (th) close();

    threadShouldExit = false;
    th = new std::thread(&VideoSet::loadFrameFromVideo, this, videoName, params);
    return true;
}

void VideoSet::loadFrameFromVideo(const std::string &videoName, const ParamSet &params) {

    cv::VideoCapture video((videoSetRoot / videoName).string());

    auto startTime = std::chrono::steady_clock::now();

    threadRunning = true;
    while (video.isOpened()) {

        uint8_t workingBuffer = 1 - lastBuffer;

        // Load the image
        cv::Mat img;
        if (threadShouldExit || !video.read(img)) {  // no more image
            bufferCaptureTime[workingBuffer] = 0;  // indicate invalid frame
            lastBuffer = workingBuffer;  // switch
            break;
        }

        // Wait for correct frame time
        auto now = std::chrono::steady_clock::now();
        auto frameTimeMS = video.get(cv::CAP_PROP_POS_MSEC);
        auto expectedTime = startTime + std::chrono::nanoseconds(
                (long long) (frameTimeMS * 1E6 / params.video_speed() / params.video_playback_speed()));
        if (now < expectedTime) {
            std::this_thread::sleep_for(expectedTime - now);
        }


        if (img.rows != params.roi_width() || img.cols != params.roi_width()) {
            cv::resize(img, img, cv::Size(params.roi_width(), params.roi_height()));
        }
        buffer[workingBuffer] = img;

        // Increment frame time, using the actual capture time
        bufferCaptureTime[workingBuffer] = (int) (frameTimeMS * 10 / params.video_speed());;

        // Switch
        lastBuffer = workingBuffer;

        // The only place of incrementing
        ++cumulativeFrameCounter;
    }
    threadRunning = false;

    std::cout << "VideoSet: closed\n";
}

void VideoSet::close() {
    if (th) {
        threadShouldExit = true;
        th->join();
        delete th;
        th = nullptr;
    }
}

}