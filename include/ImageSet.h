//
// Created by liuzikai on 5/25/21.
//

#ifndef META_VISION_SOLAIS_IMAGESET_H
#define META_VISION_SOLAIS_IMAGESET_H

#include <thread>
#include <filesystem>
#include "Parameters.h"
#include "InputSource.h"

namespace meta {

namespace fs = std::filesystem;

class ImageSet : public InputSource {
public:

    ImageSet();

    void reloadImageSetList();

    int switchImageSet(const std::string &dataSetName);

    const std::vector<std::string> &getImageSetList() const { return imageSets; }

    const std::vector<std::string> &getImageList() const { return images; }

    bool openSingleImage(const std::string &imageName, const ParamSet &params);

    bool openCurrentImageSet(const package::ParamSet &params);

    bool isOpened() const override { return !imageMats.empty(); }

    void close() override;

    TimePoint getFrameCaptureTime() const override { return bufferCaptureTime[lastBuffer]; }

    const cv::Mat &getFrame() const override { return buffer[lastBuffer]; }

    void fetchNextFrame() override;

    std::string saveCapturedImage(const cv::Mat &image, const package::ParamSet &params);

protected:

    const fs::path imageSetRoot;

    fs::path currentImageSetPath;
    std::vector<std::string> imageSets;      // directory name
    std::vector<std::string> images;         // jpg filenames
    std::vector<cv::Mat> imageMats;          // empty if not running

    // Double buffering
    bool shouldFetchNextFrame;
    uint8_t lastBuffer = 0;
    cv::Mat buffer[2];
    TimePoint bufferCaptureTime[2] = {0, 0};

    std::thread *th = nullptr;
    bool threadShouldExit = false;

    void loadFrameFromImageSet(const ParamSet &params);
};

}

#endif //META_VISION_SOLAIS_IMAGESET_H
