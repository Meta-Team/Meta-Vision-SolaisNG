//
// Created by liuzikai on 3/11/21.
//

#include "Executor.h"  // includes headers of all components
#include "TerminalSocket.h"
#include "TerminalParameters.h"
#include "Parameters.pb.h"
#include <iostream>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/cuda.hpp>
#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/color.h>

using namespace meta;
using namespace package;

/** TCP IO **/

boost::asio::io_context tcpIOContext;
std::thread *tcpIOThread = nullptr;

// Setup a server with automatic acceptance
TerminalSocketServer socketServer(tcpIOContext, 8800);


/** TCP Handling **/

// Client should only operates on the executor
std::unique_ptr<Executor> executor;

// Reuse message objects: developers.google.com/protocol-buffers/docs/cpptutorial#optimization-tips
ParamSet recvParams;
Result resultPackage;

Image *allocProtoJPG(const cv::Mat &mat) {
    auto image = new package::Image;
    image->set_format(package::Image::JPEG);

    if (!mat.empty()) {
        cv::Mat outImage;
        std::vector<uchar> buf;
        float ratio = (float) TERMINAL_IMAGE_PREVIEW_HEIGHT / (float) mat.rows;

        cv::resize(mat, outImage, cv::Size(), ratio, ratio);
        cv::imencode(".jpg", outImage, buf, {cv::IMWRITE_JPEG_QUALITY, 60});

        image->set_data(buf.data(), buf.size());
    }
    return image;
}

void sendStatusBarMsg(const std::string &msg) {
    socketServer.sendSingleString("msg", "Core: " + msg);
}

void sendResult(std::string_view mask) {

    // Always send a package, but non-empty only if the executor is running
    if (executor->hasOutputs()) {
        resultPackage.Clear();

        // Fetch outputs
        cv::Mat originalImage, brightnessImage, colorImage, lightsImage;
        std::vector<cv::RotatedRect> lightRects;
        std::vector<AimingSolver::ArmorInfo> armors;
        bool tkTriggered;
        std::deque<AimingSolver::PulseInfo> tkPulses;
        TimePoint tkPeriod;

        executor->fetchOutputs(originalImage, brightnessImage, colorImage, lightsImage, lightRects, armors,
                               tkTriggered, tkPulses, tkPeriod);
        // If can't lock immediately, simply wait. Detector only performs several non-copy assignments.

        // Detector images
        {
            // Empty handled in allocateProtoJPG
            if (mask[0] == 'T') resultPackage.set_allocated_camera_image(allocProtoJPG(originalImage));
            if (mask[1] == 'T') resultPackage.set_allocated_brightness_image(allocProtoJPG(brightnessImage));
            if (mask[2] == 'T') resultPackage.set_allocated_color_image(allocProtoJPG(colorImage));
            if (mask[3] == 'T') resultPackage.set_allocated_contour_image(allocProtoJPG(lightsImage));
        }

        float imageScale = (float) TERMINAL_IMAGE_PREVIEW_HEIGHT / executor->getCurrentParams().roi_height();

        // Light Rects
        {
            for (const auto &rect : lightRects) {
                auto r = resultPackage.add_lights();
                r->set_allocated_center(allocResultPoint2f(rect.center.x * imageScale, rect.center.y * imageScale));
                r->set_allocated_size(allocResultPoint2f(rect.size.width * imageScale, rect.size.height * imageScale));
                if (rect.angle <= 90) {
                    r->set_angle(rect.angle);
                } else {
                    r->set_angle(rect.angle - 180);
                }
            }
        }

        // Armors
        {
            for (const auto &armor : armors) {
                auto armorInfo = resultPackage.add_armors();
                for (int i = 0; i < 4; i++) {
                    auto imagePoint = armorInfo->add_image_points();
                    imagePoint->set_x(armor.imgPoints[i].x * imageScale);
                    imagePoint->set_y(armor.imgPoints[i].y * imageScale);
                }
                armorInfo->set_allocated_image_center(allocResultPoint2f(
                        armor.imgCenter.x * imageScale, armor.imgCenter.y * imageScale));
                armorInfo->set_allocated_offset(allocResultPoint3f(armor.offset.x, armor.offset.y, armor.offset.z));
                armorInfo->set_large_armor(armor.largeArmor);
                armorInfo->set_number(armor.number);
                armorInfo->set_selected(armor.flags & AimingSolver::ArmorInfo::SELECTED_TARGET);
                armorInfo->set_allocated_ypd(allocResultPoint3f(armor.ypd.x, armor.ypd.y, armor.ypd.z));
            }
        }

        // TopKiller
        {
            resultPackage.set_tk_triggered(tkTriggered);
            for (const auto &pulse : tkPulses) {
                auto p = resultPackage.add_tk_pulses();
                p->set_allocated_mid_ypd(allocResultPoint3f(pulse.ypdMid.x, pulse.ypdMid.y, pulse.ypdMid.z));
                p->set_avg_time(pulse.avgTime / 10);
                p->set_frame_count(pulse.frameCount);
            }
            resultPackage.set_tk_period(tkPeriod / 10);
        }

        // Aiming
        {
            AimingSolver::ControlCommand command;
            if (executor->aimingSolver()->getControlCommand(command)) {
                resultPackage.set_allocated_aiming_target(allocResultPoint2f(command.yawDelta, command.pitchDelta));
//                resultPackage.set_remaining_time_to_target(command.remainingTimeToTarget);
            }
        }
        socketServer.sendBytes("res", resultPackage);

    } else {

        socketServer.sendBytes("res", nullptr, 0);
    }

}

void handleRecvSingleString(std::string_view name, std::string_view s) {
    if (name == "fetch") {
        sendResult(s);  // reply anyway, whether the executor is running or not

    } else if (name == "switchImageSet") {
        if (executor->switchImageSet(std::string(s)) == 0) {
            sendStatusBarMsg("empty data set " + std::string(s));
        } else {
            socketServer.sendListOfStrings("imageList", executor->imageSet()->getImageList());
            sendStatusBarMsg("image set \"" + std::string(s) + "\" loaded");
        }

    } else if (name == "runImage") {
        executor->startSingleImageDetection(std::string(s));
        socketServer.sendSingleString("executionStarted", "image " + std::string(s));

    } else if (name == "runVideo") {
        executor->startVideoDetection(std::string(s));
        socketServer.sendSingleString("executionStarted", "video " + std::string(s));

    } else if (name == "switchParamSet") {
        executor->switchParamSet(std::string(s));
        sendStatusBarMsg("Switched to parameter set \"" + std::string(s) + "\"");

    } else if (name == "previewVideo") {
        auto img = executor->getVideoPreview(std::string(s));
        resultPackage.Clear();
        resultPackage.set_allocated_camera_image(allocProtoJPG(img));
        socketServer.sendBytes("res", resultPackage);

    } else goto INVALID_COMMAND;

    return;
    INVALID_COMMAND:
    spdlog::error("Invalid single-string package <{}> \"{}\"", name, s);
}

void handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size) {
    if (name == "fps") {
        socketServer.sendListOfStrings("fps", {
                std::to_string(executor->fetchAndClearInputFrameCounter()),
                std::to_string(executor->fetchAndClearExecutorFrameCounter()),
                std::to_string(executor->fetchAndClearSerialFrameCounter()),
        });

    } else if (name == "setParams") {
        if (!recvParams.ParseFromArray(buf, size)) {
            sendStatusBarMsg("invalid ParamSet package");
        } else {
            executor->saveAndApplyParams(recvParams);
            sendStatusBarMsg(
                    "parameter set \"" + executor->dataManager()->currentParamSetName() + "\" saved and applied");
        }

    } else if (name == "getParams") {
        socketServer.sendBytes("params", executor->getCurrentParams());

    } else if (name == "getCurrentParamSetName") {
        socketServer.sendSingleString("currentParamSetName", executor->dataManager()->currentParamSetName());

    } else if (name == "stop") {
        executor->stop();

    } else if (name == "runCamera") {
        if (!executor->startRealTimeDetection()) {
            sendStatusBarMsg("failed to start real time detection on camera");
        } else {
            socketServer.sendSingleString("executionStarted", "camera");

            // Send camera info
            resultPackage.Clear();
            resultPackage.set_camera_info(executor->camera()->getCameraInfo());
            socketServer.sendBytes("res", resultPackage);
        }

    } else if (name == "runImageSet") {
        if (!executor->startImageSetDetection()) {
            sendStatusBarMsg("failed to start streaming on image set");
        } else {
            socketServer.sendSingleString("executionStarted", "image set");
        }

    } else if (name == "fetchLists") {
        socketServer.sendListOfStrings("imageSetList", executor->imageSet()->getImageSetList());
        socketServer.sendListOfStrings("videoList", executor->videoSet()->getVideoList());
        socketServer.sendListOfStrings("paramSetList", executor->dataManager()->getParamSetList());
        socketServer.sendSingleString("currentParamSetName", executor->dataManager()->currentParamSetName());
        socketServer.sendBytes("params", executor->getCurrentParams());

    } else if (name == "reloadLists") {
        executor->reloadLists();  // switch to default parameter set

    } else if (name == "captureImage") {
        std::string filename = executor->captureImageFromCamera();
        sendStatusBarMsg("capture as " + filename);

    } else if (name == "startRecord") {
        std::string filename = executor->startRecordToVideo();
        socketServer.sendSingleString("executionStarted", "recording " + filename);

    } else if (name == "stopRecord") {
        executor->stopRecordToVideo();
        sendStatusBarMsg("stop recording video");

    } else {
        spdlog::error("Unknown bytes package <{}>", name);
    }
}

/** Serial IO **/
boost::asio::io_context serialIOContext;

/** Components **/

// TCP handling should not operates on these components directly, so they are put at last
std::unique_ptr<OpenCVCamera> openCVCamera;
std::unique_ptr<MVCamera> mvCamera;
std::unique_ptr<ImageSet> imageSet;
std::unique_ptr<VideoSet> videoSet;
std::unique_ptr<ArmorDetector> detector;
std::unique_ptr<ParamSetManager> paramSetManager;
std::unique_ptr<PositionCalculator> positionCalculator;
std::unique_ptr<AimingSolver> aimingSolver;
std::unique_ptr<Serial> serial;

int main(int argc, char *argv[]) {

    fmt::print("Welcome to {}!\n\n", PROJECT_NAME);
//    fmt::print(cv::getBuildInformation());
    fmt::print("\n"
               "     _______.  ______    __          ___       __       _______.   .__   __.   _______ \n"
               "    /       | /  __  \\  |  |        /   \\     |  |     /       |   |  \\ |  |  /  _____|\n"
               "   |   (----`|  |  |  | |  |       /  ^  \\    |  |    |   (----`   |   \\|  | |  |  __  \n"
               "    \\   \\    |  |  |  | |  |      /  /_\\  \\   |  |     \\   \\       |  . `  | |  | |_ | \n"
               ".----)   |   |  `--'  | |  `----./  _____  \\  |  | .----)   |      |  |\\   | |  |__| | \n"
               "|_______/     \\______/  |_______/__/     \\__\\ |__| |_______/       |__| \\__|  \\______| \n"
               "                                                                                       \n");

    CameraSdkInit(0);   // for MVCamera

    openCVCamera = std::make_unique<OpenCVCamera>();
    mvCamera = std::make_unique<MVCamera>();
    imageSet = std::make_unique<ImageSet>();
    videoSet = std::make_unique<VideoSet>();
    detector = std::make_unique<ArmorDetector>();
    paramSetManager = std::make_unique<ParamSetManager>();
    positionCalculator = std::make_unique<PositionCalculator>();
    aimingSolver = std::make_unique<AimingSolver>();
    if (strlen(SERIAL_DEVICE) != 0) {
        serial = std::make_unique<Serial>(serialIOContext);
    } else {
        spdlog::warn("Solais: Serial connection disabled for debug purposes");
    }
    executor = std::make_unique<Executor>(openCVCamera.get(), mvCamera.get(), imageSet.get(), videoSet.get(),
                                          paramSetManager.get(),
                                          detector.get(), positionCalculator.get(), aimingSolver.get(),
                                          serial.get());

    tcpIOThread = new std::thread([] {
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> workGuard(tcpIOContext.get_executor());
        tcpIOContext.run();  // this operation is blocking, until ioContext is deleted
    });

    socketServer.startAccept();
    socketServer.setCallbacks(handleRecvSingleString,
                              nullptr,
                              handleRecvBytes,
                              nullptr);

    // Start on camera
    executor->startRealTimeDetection();

    // Main thread is then used for serial IO
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> workGuard(serialIOContext.get_executor());
    serialIOContext.run();  // this operation is blocking, until ioContext is deleted

    // Shall not get to here
    tcpIOThread->join();

    return 0;
}