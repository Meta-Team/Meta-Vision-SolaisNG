//
// Created by liuzikai on 6/27/21.
//

#ifndef META_VISION_SOLAIS_VIDEOSET_H
#define META_VISION_SOLAIS_VIDEOSET_H

#include <thread>
#include <filesystem>
#include "Parameters.h"
#include "InputSource.h"
#include "Utilities.h"

namespace meta {

namespace fs = std::filesystem;

class VideoSet : public InputSource {
public:

    VideoSet();

    void reloadVideoList();

    const std::vector<std::string> &getVideoList() const { return videos; }

    cv::Mat getVideoFirstFrame(const std::string &videoName, const ParamSet &params) const;

    bool openVideo(const std::string &videoName, const ParamSet &params);

    bool isOpened() const override { return threadRunning; }

    void close() override;

    TimePoint getFrameCaptureTime() const override { return bufferCaptureTime[lastBuffer]; }

    const cv::Mat &getFrame() const override { return buffer[lastBuffer]; }

    const fs::path videoSetRoot;

protected:

    std::vector<std::string> videos;

    // Double buffering
    uint8_t lastBuffer = 0;
    cv::Mat buffer[2];
    TimePoint bufferCaptureTime[2] = {0, 0};

    std::thread *th = nullptr;
    bool threadShouldExit = false;
    bool threadRunning = false;

    void loadFrameFromVideo(const std::string &videoName, const ParamSet &params);
};

}

#endif //META_VISION_SOLAIS_VIDEOSET_H
