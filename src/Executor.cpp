//
// Created by liuzikai on 4/18/21.
//

#include "Executor.h"
#include "Utilities.h"
#include <iostream>

namespace meta {

Executor::Executor(OpenCVCamera *openCvCamera, MVCamera *mvCamera, ImageSet *imageSet, VideoSet *videoSet,
                   ParamSetManager *paramSetManager,
                   ArmorDetector *detector, PositionCalculator *positionCalculator, AimingSolver *aimingSolver,
                   Serial *serial)
        : openCvCamera_(openCvCamera), mvCamera_(mvCamera), imageSet_(imageSet), videoSet_(videoSet),
          paramSetManager_(paramSetManager),
          detector_(detector), positionCalculator_(positionCalculator), aimingSolver_(aimingSolver),
          serial_(serial) {

    reloadLists();
}

void Executor::switchParamSet(const std::string &paramSetName) {
    paramSetManager_->switchToParamSet(paramSetName);
    applyParams(paramSetManager_->loadCurrentParamSet());
}

void Executor::saveAndApplyParams(const ParamSet &p) {
    paramSetManager_->saveCurrentParamSet(p);
    applyParams(p);
}

void Executor::applyParams(const ParamSet &p) {
    // For better user experience for parameter tuning, here we don't stop the detection thread
    // But if there is some changes in the streaming source, detection may terminate anyway

    // Input of Camera
    // Skip re-opening the video source if the parameter doesn't change to save some time
    if (!(p.camera_backend() == params.camera_backend() && p.camera_id() == params.camera_id() &&
          p.image_width() == params.image_width() && p.image_height() == params.image_height() &&
          p.fps() == params.fps() &&
          p.gamma().enabled() == params.gamma().enabled() && p.gamma().val() == params.gamma().val() &&
          p.roi_width() == params.roi_width() && p.roi_height() == params.roi_height() &&
          p.manual_exposure().enabled() == params.manual_exposure().enabled() &&
          p.manual_exposure().val() == params.manual_exposure().val())) {

        bool cameraOpened = camera_ && camera_->isOpened();
        if (cameraOpened) camera_->close();
        if (p.camera_backend() == ParamSet::MV_CAMERA) {
            camera_ = mvCamera_;
            std::cout << "Executor: use MVCamera" << std::endl;
        } else {
            camera_ = openCvCamera_;
            std::cout << "Executor: use OpenCVCamera" << std::endl;
        }
        if (cameraOpened) camera_->open(p);
    }

    // Local copy
    params = p;

    // Input of ImageSet
    if (imageSet_->isOpened()) {
        stop();
        imageSet_->close();
    }

    // Detector
    detector_->setParams(params);

    // PositionCalculator
    {
        std::string filename =
                std::string(PARAM_SET_ROOT) + "/params/" +
                std::to_string(params.image_width()) + "x" + std::to_string(params.image_height()) + ".xml";

        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        float zScale;

        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "Failed to open " << filename << std::endl;
            std::exit(1);
        }

        fs["cameraMatrix"] >> cameraMatrix;
        cameraMatrix.at<double>(0, 2) *= (float) params.roi_width() / (float) params.image_width();
        cameraMatrix.at<double>(1, 2) *= (float) params.roi_height() / (float) params.image_height();
        fs["distCoeffs"] >> distCoeffs;
        fs["zScale"] >> zScale;

        positionCalculator_->setParameters(
                {(float) params.small_armor_size().x(), (float) params.small_armor_size().y()},
                {(float) params.large_armor_size().x(), (float) params.large_armor_size().y()},
                cameraMatrix, distCoeffs, zScale);
    }

    // AimingSolver
    aimingSolver_->setParams(params);  // reset history inside
}

void Executor::stop() {
    if (th) {  // executor is running
        threadShouldExit = true;
        th->join();
        delete th;
        th = nullptr;
    }
    curAction = NONE;
}

bool Executor::startRealTimeDetection() {
    if (th) stop();

    if (!camera_) return false;
    if (!camera_->isOpened()) {
        if (!camera_->open(params)) {
            return false;
        }
    }

    // Start real-time detection thread
    curAction = STREAMING_DETECTION;
    threadShouldExit = false;
    th = new std::thread(&Executor::runStreamingDetection, this, camera_);
    return true;
}

bool Executor::startImageSetDetection() {
    if (th) stop();

    if (imageSet_->isOpened()) imageSet_->close();  // always restart
    if (!imageSet_->openCurrentImageSet(params)) return false;

    // Start real-time detection thread
    curAction = STREAMING_DETECTION;
    threadShouldExit = false;
    th = new std::thread(&Executor::runStreamingDetection, this, imageSet_);
    return true;
}

bool Executor::startSingleImageDetection(const std::string &imageName) {
    if (th) stop();

    if (imageSet_->isOpened()) imageSet_->close();
    if (!imageSet_->openSingleImage(imageName, params)) return false;

    curAction = SINGLE_IMAGE_DETECTION;
    threadShouldExit = false;
    th = new std::thread(&Executor::runStreamingDetection, this, imageSet_);
    return true;
}

bool Executor::startVideoDetection(const std::string &videoName) {
    if (th) stop();

    if (videoSet_->isOpened()) videoSet_->close();
    if (!videoSet_->openVideo(videoName, params)) return false;

    curAction = STREAMING_DETECTION;
    threadShouldExit = false;
    th = new std::thread(&Executor::runStreamingDetection, this, videoSet_);
    return true;
}

void Executor::runStreamingDetection(InputSource *source) {
    std::cout << "Executor: start streaming\n";

    currentInput_ = source;
    currentInput_->fetchAndClearFrameCounter();
    aimingSolver_->resetHistory();

    TimePoint lastFrameTime = 0;  // use last frame capture time to wait for new frame
    TimePoint frameTime = 0;
    while (true) {

        // Wait for new frame, fetch and store time first and then compare
        while (!threadShouldExit && lastFrameTime == (frameTime = source->getFrameCaptureTime())) {
            source->fetchNextFrame();
            std::this_thread::yield();
        }

        if (threadShouldExit || frameTime == 0) {
            break;
        }
        lastFrameTime = frameTime;

        auto &img = source->getFrame();  // no need for deep copying
        source->fetchNextFrame();


        // Run armor detection algorithm
        // For the compile on no CUDA supported platforms
#ifdef ON_JETSON
        std::vector<ArmorDetector::DetectedArmor> detectedArmors = detector_->detect_NG(img);
#else
        std::vector<ArmorDetector::DetectedArmor> detectedArmors = detector_->detect(img);
#endif

        // Solve armor positions
        std::vector<AimingSolver::ArmorInfo> armors;
        for (const auto &detectedArmor : detectedArmors) {
            cv::Point3f offset;
            float longLightLength = std::max(cv::norm(detectedArmor.points[1] - detectedArmor.points[0]),
                                             cv::norm(detectedArmor.points[2] - detectedArmor.points[3]));
            if (positionCalculator_->solve(detectedArmor.points,
                                           detectedArmor.largeArmor,
                                           params.manual_pnp_rect_max_height().enabled() &&
                                           (longLightLength < params.manual_pnp_rect_max_height().val()),
                                           offset)) {
                armors.emplace_back(AimingSolver::ArmorInfo{
                        detectedArmor.points,
                        detectedArmor.center,
                        offset,
                        detectedArmor.avgLightAngle,
                        detectedArmor.largeArmor,
                        detectedArmor.number
                });
            }
        }

        // Update
        aimingSolver_->updateArmors(armors, frameTime);

        AimingSolver::ControlCommand command;
        if (serial_ && aimingSolver_->getControlCommand(command)) {
            // Send control command
            serial_->sendControlCommand(
                    command.detected,
                    command.topKillerTriggered,
                    frameTime,
                    command.yawDelta,
                    command.pitchDelta,
                    command.dist,
                    command.avgLightAngle,
                    command.imageX,
                    command.imageY,
                    command.remainingTimeToTarget,
                    command.period);
        }

        // Assign (no copying for cv::Mat) results all at once, if the result is not being processed
        if (outputMutex.try_lock()) {
            originalOutput = detector_->imgOriginal;
            brightnessOutput = detector_->imgBrightness;
            colorOutput = detector_->imgColor;
            lightsImageOutput = detector_->imgLights;
            lightRectsOutput = detector_->lightRects;
            armorsOutput = armors;
            tkTriggeredOutput = aimingSolver_->topKiller.triggered;
            tkPulsesOutput = aimingSolver_->topKiller.pulses;
            tkPeriodOutput = aimingSolver_->topKiller.period;

            outputMutex.unlock();
        }
        // Otherwise, simple discard the results of current run

        // Increment frame counter
        cumulativeFrameCounter++;
    }

    std::cout << "Executor: stopped\n";

    source->close();
    currentInput_ = nullptr;
    if (curAction != SINGLE_IMAGE_DETECTION) {  // do not reset SINGLE_IMAGE_DETECTION for result fetching
        curAction = NONE;
    }
}

void Executor::reloadLists() {
    imageSet_->reloadImageSetList();
    videoSet_->reloadVideoList();
    paramSetManager_->reloadParamSetList();  // switch to default parameter set
    applyParams(paramSetManager_->loadCurrentParamSet());
}

int Executor::switchImageSet(const std::string &path) {
    return imageSet_->switchImageSet(path);
}

unsigned int Executor::fetchAndClearInputFrameCounter() {
    InputSource *input = currentInput_;  // make a copy
    if (input) {
        return input->fetchAndClearFrameCounter();
    } else {
        if (camera_ && camera_->isRecordingVideo()) {
            return camera_->fetchAndClearFrameCounter();
        } else {
            return 0;
        }
    }
}

std::string Executor::captureImageFromCamera() {
    if (!camera_) return "[Error: camera backend not set]";
    if (!camera_->isOpened()) {
        if (!camera_->open(params)) {
            return "[Error: failed to open camera]";
        }
    }
    cv::Mat img = camera_->getFrame();  // no actual data copy
    return imageSet_->saveCapturedImage(img, params);
}

std::string Executor::startRecordToVideo() {
    if (!camera_) return "[Error: camera backend not set]";
    if (!camera_->isOpened()) {
        if (!camera_->open(params)) {
            return "[Error: failed to open camera]";
        }
    }
    if (camera_->getFPS() == 0) {
        return "[Error: camera reported 0 FPS]";
    }

    std::string filename = videoSet_->videoSetRoot.string();
    if (!camera_->startRecordToVideo(filename, params)) {
        return "[Error: failed to open videoWriter]";
    }

    return filename;
}

bool Executor::hasOutputs() {
    if (curAction == SINGLE_IMAGE_DETECTION) {
        curAction = NONE;  // reset
        return true;
    }
    if (curAction != NONE) return true;
    if (camera_ && camera_->isRecordingVideo()) return true;
    return false;
}

void Executor::fetchOutputs(cv::Mat &originalImage, cv::Mat &brightnessImage, cv::Mat &colorImage,
                            cv::Mat &lightsImage, std::vector<cv::RotatedRect> &lightRects,
                            std::vector<AimingSolver::ArmorInfo> &armors,
                            bool &tkTriggered, std::deque<AimingSolver::PulseInfo> &tkPulses, TimePoint &tkPeriod) {
    if (curAction != NONE) {
        outputMutex.lock();
        {
            originalImage = originalOutput;
            brightnessImage = brightnessOutput;
            colorImage = colorOutput;
            lightsImage = lightsImageOutput;
            lightRects = lightRectsOutput;
            armors = armorsOutput;
            tkTriggered = tkTriggeredOutput;
            tkPulses = tkPulsesOutput;
            tkPeriod = tkPeriodOutput;
        }
        outputMutex.unlock();

    } else {
        if (camera_ && camera_->isRecordingVideo()) {
            originalImage = camera_->getFrame();
        }
    }
}

}