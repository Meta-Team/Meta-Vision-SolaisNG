//
// Created by liuzikai on 3/11/21.
//

#ifndef META_VISION_SOLAIS_CAMERA_H
#define META_VISION_SOLAIS_CAMERA_H

#include "Common.h"
#include <thread>
#include <atomic>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "Parameters.pb.h"

namespace meta {

class Camera {
public:

    explicit Camera() : lastBuffer(0) {}

    ~Camera();

    void open(const package::ParamSet &params);

    bool isOpened() const { return cap.isOpened(); }

    string getCapInfo() const { return capInfoSS.str(); };

    void release();

    unsigned int getFrameID() const { return bufferFrameID[lastBuffer]; }

    const cv::Mat &getFrame() const { return buffer[lastBuffer]; }

    using NewFrameCallBack = void (*)(void *);

    void registerNewFrameCallBack(NewFrameCallBack callBack, void *param);


private:

    cv::VideoCapture cap;

    std::stringstream capInfoSS;

    // Double buffering
    std::atomic<int> lastBuffer;
    cv:: Mat buffer[2];
    unsigned int bufferFrameID[2] = {0, 0};

    std::thread *th = nullptr;
    std::atomic<bool> threadShouldExit;

    std::vector<std::pair<NewFrameCallBack, void *>> callbacks;

    void readFrameFromCamera(const package::ParamSet &params);

};

}

#endif //META_VISION_SOLAIS_CAMERA_H
