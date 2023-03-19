//
// Created by liuzikai on 5/25/21.
//

#include "ImageSet.h"
#include "Utilities.h"
#include <iostream>
#include <iomanip>
#include <opencv2/imgproc/imgproc.hpp>

namespace meta {

// DATA_SET_ROOT defined in CMakeLists.txt
ImageSet::ImageSet() : imageSetRoot(fs::path(DATA_SET_ROOT) / "images") {
    if (!fs::exists(imageSetRoot)) {
        fs::create_directories(imageSetRoot);
    }
}

void ImageSet::reloadImageSetList() {
    imageSets.clear();
    currentImageSetPath = "";
    images.clear();
    if (fs::is_directory(imageSetRoot)) {
        for (const auto &entry : fs::directory_iterator(imageSetRoot)) {
            if (fs::is_directory(imageSetRoot / entry.path().filename())) {
                imageSets.emplace_back(entry.path().filename().string());
            }
        }
    }
}

int ImageSet::switchImageSet(const std::string &dataSetName) {
    if (isOpened()) close();

    currentImageSetPath = imageSetRoot / dataSetName;
    images.clear();

    std::cout << "ImageSet: loading data set " << currentImageSetPath << "..." << std::endl;

    for (const auto &entry : fs::directory_iterator(currentImageSetPath)) {
#ifdef BOOST_OS_WINDOWS
        if (wcscmp(entry.path().extension().c_str(), L".jpg") == 0) {
#else
        if (strcasecmp(entry.path().extension().c_str(), ".jpg") == 0) {
#endif
            fs::path xmlFile = fs::path(currentImageSetPath) / entry.path().stem();
            xmlFile += ".xml";

            if (!fs::exists(xmlFile)) {
                std::cerr << "Missing xml file: " << entry.path().filename() << std::endl;
                // continue;
            }

            images.emplace_back(entry.path().filename().string());
        }
    }

    std::sort(images.begin(), images.end());

    std::cout << "ImageSet: " << images.size() << " images loaded." << std::endl;

    return images.size();
}

bool ImageSet::openSingleImage(const std::string &imageName, const ParamSet &params) {
    if (currentImageSetPath.empty()) {
        std::cerr << "ImageSet: failed to open as no image set is selected\n";
        return false;
    }

    fs::path imageFile = fs::path(currentImageSetPath) / imageName;
    auto img = cv::imread(imageFile.string());
    if (img.rows != params.roi_height() || img.cols != params.roi_width()) {
        cv::resize(img, img, cv::Size(params.roi_width(), params.roi_height()));
    }

    // Load the image to the latest buffer
    buffer[lastBuffer] = img;
    bufferCaptureTime[lastBuffer] = 1;

    if (th) close();
    th = nullptr;  // do not start thread and clear the pointer for fetchNextFrame
    return true;
}

bool ImageSet::openCurrentImageSet(const ParamSet &params) {
    if (th) close();

    if (currentImageSetPath.empty()) {
        std::cerr << "ImageSet: failed to open as no image set is selected\n";
        return false;
    }

    std::cout << "ImageSet: loading " << images.size() << " images into memory\n";
    imageMats.clear();
    imageMats.reserve(images.size());
    for (const auto &image : images) {
        fs::path imageFile = fs::path(currentImageSetPath) / image;
        auto img = cv::imread(imageFile.string());
        if (img.rows != params.roi_height() || img.cols != params.roi_width()) {
            cv::resize(img, img, cv::Size(params.roi_width(), params.roi_height()));
        }
        imageMats.emplace_back(img);
    }

    threadShouldExit = false;
    th = new std::thread(&ImageSet::loadFrameFromImageSet, this, params);
    return true;
}

void ImageSet::loadFrameFromImageSet(const ParamSet &params) {
    shouldFetchNextFrame = true;

    auto it = imageMats.begin();  // next frame iterator
    while (true) {

        while (!shouldFetchNextFrame && !threadShouldExit) std::this_thread::yield();

        uint8_t workingBuffer = 1 - lastBuffer;

        if (threadShouldExit || it == imageMats.end()) {  // no more image
            bufferCaptureTime[workingBuffer] = 0;  // indicate invalid frame
            lastBuffer = workingBuffer;  // switch
            break;
        }

        // Set the image
        buffer[workingBuffer] = *it;
        ++it;

        // Increment frame time
        bufferCaptureTime[workingBuffer] = bufferCaptureTime[lastBuffer] + 1;

        // Switch
        lastBuffer = workingBuffer;

        // The only place of incrementing
        ++cumulativeFrameCounter;

        shouldFetchNextFrame = false;
    }

    imageMats.clear();
    std::cout << "ImageSet: closed\n";
}


void ImageSet::fetchNextFrame() {

    if (th) {  // if the thread is created

        while (shouldFetchNextFrame) {  // check for flag set last time
            std::this_thread::yield();
        }
        shouldFetchNextFrame = true;

    } else {  // for single image, the thread is not created

        uint8_t workingBuffer = 1 - lastBuffer;
        bufferCaptureTime[workingBuffer] = 0;  // indicate invalid frame
        lastBuffer = workingBuffer;  // switch
    }
}

void ImageSet::close() {
    if (th) {
        threadShouldExit = true;
        th->join();
        delete th;
        th = nullptr;
    }
}

std::string ImageSet::saveCapturedImage(const cv::Mat &image, const package::ParamSet &params) {
    // Directory: <width>_<height>_blue/red
    fs::path dir = imageSetRoot /
                   fs::path(std::to_string(params.roi_width()) + "_" + std::to_string(params.roi_height()) + "_" +
                            (params.enemy_color() == package::ParamSet_EnemyColor_BLUE ? "blue" : "red"));

    if (!fs::exists(dir)) {
        fs::create_directories(dir);
    }

    // Filename: formatted current time with milliseconds
    fs::path basename = dir / fs::path(currentTimeString());
    fs::path filename = basename;
    filename += ".jpg";
    if (fs::exists(filename)) {
        int i = -1;
        do {
            i++;
            filename = basename;
            filename += "_" + std::to_string(i) + ".jpg";
        } while (fs::exists(filename));
    }

    // Save image
    cv::imwrite(filename.string(), image);

    // Let the user reload the image list

    return filename.string();
}

}