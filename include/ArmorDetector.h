//
// Created by liuzikai on 2/6/21.
//

#ifndef META_VISION_SOLAIS_ARMORDETECTOR_H
#define META_VISION_SOLAIS_ARMORDETECTOR_H

#include "Parameters.h"
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef ON_JETSON
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include "YOLOv5_TensorRT.h"
#endif

namespace meta {

class ArmorDetector {
public:

#ifdef ON_JETSON
    ArmorDetector() : yoloModel(std::string("/home/nvidia/tmp/tmp.pS4QeSxQaM/nn-models/model-opt-4.onnx")) {
    }
#endif
    void setParams(const ParamSet &p) { params = p; }

    const ParamSet &getParams() const { return params; }

    struct DetectedArmor {
        std::array<cv::Point2f, 4> points;
        cv::Point2f center;
        bool largeArmor = false;
        int number = 0;                 // not implemented yet
        std::array<int, 2> lightIndex;  // left, right ; already deprecated after YOLO model
        float lightAngleDiff;           // absolute value, non-negative
        float avgLightAngle;
    };

    std::vector<DetectedArmor> detect(const cv::Mat &img);
    std::vector<DetectedArmor> detect_NG(const cv::Mat &img);

    static float normalizeLightAngle(float angle) { return angle <= 90 ? angle : 180 - angle; }

private:

    ParamSet params;

    cv::Mat imgOriginal;
    cv::Mat imgGray;
    cv::Mat imgBrightness;
    cv::Mat imgColor;
    std::vector<cv::RotatedRect> lightRects;
    cv::Mat imgLights;
#ifdef ON_JETSON
    YOLODet yoloModel;
#endif
    static void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &boarderColor);

    /**
     * Canonicalize a non-square rotated rect from cv::minAreaRect and make:
     *  width: the short edge
     *  height: the long edge
     *  angle: in [0, 180). The angle is then the angle between the long edge and the vertical axis.
     *  https://stackoverflow.com/questions/22696539/reorder-four-points-of-a-rectangle-to-the-correct-order
     * @param rect
     */
    static void canonicalizeRotatedRect(cv::RotatedRect &rect);

    std::vector<DetectedArmor>::iterator filterAcceptedArmorsToRemove(std::vector<DetectedArmor> &acceptedArmors) const;

    friend class Executor;

};

}

#endif //META_VISION_SOLAIS_ARMORDETECTOR_H
