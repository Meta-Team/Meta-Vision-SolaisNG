//
// Created by liuzikai on 2/6/21.
//

#ifndef META_VISION_SOLAIS_ARMORDETECTOR_H
#define META_VISION_SOLAIS_ARMORDETECTOR_H

#include "Parameters.h"
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace meta {

class ArmorDetector {
public:

    void setParams(const ParamSet &p) { params = p; }

    const ParamSet &getParams() const { return params; }

    struct DetectedArmor {
        std::array<cv::Point2f, 4> points;
        cv::Point2f center;
        bool largeArmor = false;
        int number = 0;
    };

    std::vector<DetectedArmor> detect(const cv::Mat &img);

    void clearImages() { imgOriginal = imgGray = imgBrightness = imgColor = imgLights = cv::Mat(); }

private:

    ParamSet params;

    cv::Mat imgOriginal;
    cv::Mat imgGray;
    cv::Mat imgBrightness;
    cv::Mat imgColor;
    cv::Mat imgLights;
    cv::Mat imgContours;

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

    friend class Executor;

};

}

#endif //META_VISION_SOLAIS_ARMORDETECTOR_H
