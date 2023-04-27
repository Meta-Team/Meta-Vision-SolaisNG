//
// Created by liuzikai on 2/6/21.
//

#include "ArmorDetector.h"
#include <spdlog/spdlog.h>

using namespace cv;

namespace meta {

/**
 * @brief detect armor
 * @param img: input image
 * @return detected armors
 *
 * @deprecated use detect_NG(const cv::Mat &img) instead
 */
std::vector<ArmorDetector::DetectedArmor> ArmorDetector::detect(const Mat &img) {

    /*
     * Note: in this mega function, steps are wrapped with {} to reduce local variable pollution and make it easier to
     *       read. Please follow the convention.
     * Note: for debug purpose, allows some local variables and single-line compound statements.
     */

    // ================================ Setup ================================
    {
        imgOriginal = img;
        imgGray = imgBrightness = imgColor = imgLights = Mat();
    }

    // ================================ Brightness Threshold ================================
    {
        cvtColor(imgOriginal, imgGray, COLOR_BGR2GRAY);
        threshold(imgGray, imgBrightness, params.brightness_threshold(), 255, THRESH_BINARY);
    }

    // ================================ Color Threshold ================================
    {
        if (params.color_threshold_mode() == ParamSet::HSV) {

            // Convert to HSV color space
            Mat hsvImg;
            cvtColor(imgOriginal, hsvImg, COLOR_BGR2HSV);

            if (params.enemy_color() == ParamSet::RED) {
                // Red color spreads over the 0 (180) boundary, so combine them
                Mat thresholdImg0, thresholdImg1;
                inRange(hsvImg, Scalar(0, 0, 0), Scalar(params.hsv_red_hue().max(), 255, 255), thresholdImg0);
                inRange(hsvImg, Scalar(params.hsv_red_hue().min(), 0, 0), Scalar(180, 255, 255), thresholdImg1);
                imgColor = thresholdImg0 | thresholdImg1;
            } else {
                inRange(hsvImg, Scalar(params.hsv_blue_hue().min(), 0, 0),
                        Scalar(params.hsv_blue_hue().max(), 255, 255),
                        imgColor);
            }

        } else {

            std::vector<Mat> channels;
            split(imgOriginal, channels);

            // Filter using channel subtraction
            int mainChannel = (params.enemy_color() == ParamSet::RED ? 2 : 0);
            int oppositeChannel = (params.enemy_color() == ParamSet::RED ? 0 : 2);
            subtract(channels[mainChannel], channels[oppositeChannel], imgColor);
            threshold(imgColor, imgColor, params.rb_channel_threshold(), 255, THRESH_BINARY);

        }

        // Color erode
        if (params.contour_erode().enabled()) {
            Mat element = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(params.contour_erode().val(), params.contour_erode().val()));
            erode(imgColor, imgColor, element);
        }

        // Color dilate
        if (params.contour_dilate().enabled()) {
            Mat element = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(params.contour_dilate().val(), params.contour_dilate().val()));
            dilate(imgColor, imgColor, element);
        }

        // Apply filter
        imgLights = imgBrightness & imgColor;
    }

    // ================================ Find Contours ================================

    // Contour open
    if (params.contour_open().enabled()) {
        Mat element = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(params.contour_open().val(), params.contour_open().val()));
        morphologyEx(imgLights, imgLights, MORPH_OPEN, element);
    }

    // Contour close
    if (params.contour_close().enabled()) {
        Mat element = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(params.contour_close().val(), params.contour_close().val()));
        morphologyEx(imgLights, imgLights, MORPH_CLOSE, element);
    }

    {
        lightRects.clear();

        std::vector<std::vector<Point>> contours;
        findContours(imgLights, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Filter individual contours
        for (const auto &contour : contours) {

            // Filter pixel count
            if (params.contour_pixel_count().enabled()) {
                if (contour.size() < params.contour_pixel_count().val()) {
                    continue;
                }
            }

            // Filter area size
            if (params.contour_min_area().enabled()) {
                double area = contourArea(contour);
                if (area < params.contour_min_area().val()) {
                    continue;
                }
            }

            // Fit contour using a rotated rect
            RotatedRect rect;
            switch (params.contour_fit_function()) {
                case ParamSet::MIN_AREA_RECT:
                    rect = minAreaRect(contour);
                    break;
                case ParamSet::ELLIPSE:
                    // There should be at least 5 points to fit the ellipse
                    if (contour.size() < 5) continue;
                    rect = fitEllipse(contour);
                    break;
                case ParamSet::ELLIPSE_AMS:
                    if (contour.size() < 5) continue;
                    rect = fitEllipseAMS(contour);
                    break;
                case ParamSet::ELLIPSE_DIRECT:
                    if (contour.size() < 5) continue;
                    rect = fitEllipseDirect(contour);
                    break;
                default:
                    assert(!"Invalid params.contour_fit_function()");
            }
            canonicalizeRotatedRect(rect);
            // Now, width: the short edge, height: the long edge, angle: in [0, 180)

            // Filter long edge min length
            if (params.long_edge_min_length().enabled() && rect.size.height < params.long_edge_min_length().val()) {
                continue;
            }

            // Filter angle
            if (params.light_max_rotation().enabled() &&
                std::min(rect.angle, 180 - rect.angle) >= params.light_max_rotation().val()) {
                continue;
            }

            // Filter aspect ratio
            if (params.light_aspect_ratio().enabled()) {
                double aspectRatio = rect.size.height / rect.size.width;
                if (!inRange(aspectRatio, params.light_aspect_ratio())) {
                    continue;
                }
            }

            // Accept the rect
            lightRects.emplace_back(rect);
        }
    }

    // If there is less than two light contours, stop detection
    if (lightRects.size() < 2) {
        return {};
    }

    // Sort lights from left to right based on center X
    sort(lightRects.begin(), lightRects.end(),
         [](RotatedRect &a1, RotatedRect &a2) {
             return a1.center.x < a2.center.x;
         });

    /*
     * OpenCV coordinate: +x right, +y down
     */

    // ================================ Combine Lights to Armors ================================
    std::vector<DetectedArmor> acceptedArmors;
    {
        std::array<Point2f, 4> armorPoints;
        /*
         *              1 ----------- 2
         *            |*|             |*|
         * left light |*|             |*| right light
         *            |*|             |*|
         *              0 ----------- 3
         *
         * Edges (0, 1) and (2, 3) lie on inner edge
         */

        for (int leftLightIndex = 0; leftLightIndex < lightRects.size() - 1; ++leftLightIndex) {

            const RotatedRect &leftRect = lightRects[leftLightIndex];  // already canonicalized


            Point2f leftPoints[4];
            leftRect.points(leftPoints);  // bottomLeft, topLeft, topRight, bottomRight of unrotated rect
            if (leftRect.angle <= 90) {
                armorPoints[0] = (leftPoints[0] + leftPoints[3]) / 2;
                armorPoints[1] = (leftPoints[1] + leftPoints[2]) / 2;
            } else {
                armorPoints[0] = (leftPoints[1] + leftPoints[2]) / 2;
                armorPoints[1] = (leftPoints[0] + leftPoints[3]) / 2;
            }

            auto &leftCenter = leftRect.center;

            for (int rightLightIndex = leftLightIndex + 1; rightLightIndex < lightRects.size(); rightLightIndex++) {

                const RotatedRect &rightRect = lightRects[rightLightIndex];  // already canonicalized


                Point2f rightPoints[4];
                rightRect.points(rightPoints);  // bottomLeft, topLeft, topRight, bottomRight of unrotated rect
                if (rightRect.angle <= 90) {
                    armorPoints[3] = (rightPoints[0] + rightPoints[3]) / 2;
                    armorPoints[2] = (rightPoints[1] + rightPoints[2]) / 2;
                } else {
                    armorPoints[3] = (rightPoints[1] + rightPoints[2]) / 2;
                    armorPoints[2] = (rightPoints[0] + rightPoints[3]) / 2;
                }


                auto leftVector = armorPoints[1] - armorPoints[0];   // up
                if (leftVector.y > 0) {
                    continue;  // leftVector should be upward, or lights intersect
                }
                auto rightVector = armorPoints[2] - armorPoints[3];  // up
                if (rightVector.y > 0) {
                    continue;  // rightVector should be upward, or lights intersect
                }
                auto topVector = armorPoints[2] - armorPoints[1];    // right
                if (topVector.x < 0) {
                    continue;  // topVector should be rightward, or lights intersect
                }
                auto bottomVector = armorPoints[3] - armorPoints[0];  // right
                if (bottomVector.x < 0) {
                    continue;  // bottomVector should be rightward, or lights intersect
                }


                auto &rightCenter = rightRect.center;

                double leftLength = cv::norm(armorPoints[1] - armorPoints[0]);
                double rightLength = cv::norm(armorPoints[2] - armorPoints[3]);
                double averageLength = (leftLength + rightLength) / 2;

                // Filter long light length to short light length ratio
                if (params.light_length_max_ratio().enabled()) {
                    double lengthRatio = leftLength > rightLength ?
                                         leftLength / rightLength : rightLength / leftLength;  // >= 1
                    if (lengthRatio > params.light_length_max_ratio().val()) continue;
                }

                // Filter central X's difference
                if (params.light_x_dist_over_l().enabled()) {
                    double xDiffOverAvgL = abs(leftCenter.x - rightCenter.x) / averageLength;
                    if (!inRange(xDiffOverAvgL, params.light_x_dist_over_l())) {
                        continue;
                    }
                }

                // Filter central Y's difference
                if (params.light_y_dist_over_l().enabled()) {
                    double yDiffOverAvgL = abs(leftCenter.y - rightCenter.y) / averageLength;
                    if (!inRange(yDiffOverAvgL, params.light_y_dist_over_l())) {
                        continue;
                    }
                }

                // Filter angle difference
                float angleDiff = std::abs(leftRect.angle - rightRect.angle);
                if (params.light_angle_max_diff().enabled()) {
                    if (angleDiff > 90) {
                        angleDiff = 180 - angleDiff;
                    }
                    if (angleDiff > params.light_angle_max_diff().val()) {
                        continue;
                    }
                }

                double armorHeight = (cv::norm(leftVector) + cv::norm(rightVector)) / 2;
                double armorWidth = (cv::norm(topVector) + cv::norm(bottomVector)) / 2;

                // Filter armor aspect ratio
                bool largeArmor;
                if (inRange(armorWidth / armorHeight, params.small_armor_aspect_ratio())) {
                    largeArmor = false;
                } else if (inRange(armorWidth / armorHeight, params.large_armor_aspect_ratio())) {
                    largeArmor = true;
                } else {
                    continue;
                }


                // Accept the armor
                Point2f center = {0, 0};
                for (int i = 0; i < 4; i++) {
                    center.x += armorPoints[i].x;
                    center.y += armorPoints[i].y;
                }

                // Just use the average X and Y coordinate for the four point
                center.x /= 4;
                center.y /= 4;

                acceptedArmors.emplace_back(DetectedArmor{
                        armorPoints,
                        center,
                        largeArmor,
                        0,
                        {leftLightIndex, rightLightIndex},
                        angleDiff,
                        (normalizeLightAngle(leftRect.angle) + normalizeLightAngle(rightRect.angle)) / 2
                });
            }
        }
    }

    // Filter armors that share lights
    {
        while(true) {
            auto it = filterAcceptedArmorsToRemove(acceptedArmors);
            if (it == acceptedArmors.end()) break;  // nothing to remove
            acceptedArmors.erase(it);       // remove the armor
            // continue to try again
        }
    }

    return acceptedArmors;
}
#ifdef ON_JETSON

/**
 * @brief detect armor (with YOLOv5)
 * @param img: input image
 * @return detected armors
 */
std::vector<ArmorDetector::DetectedArmor> ArmorDetector::detect_NG(const cv::Mat &img) {
    imgOriginal = img;
    // Calculate inference time in ms, with high precision
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<YOLODet::bbox_t> detectResults = yoloModel(img);
    auto end = std::chrono::high_resolution_clock::now();
    double inferenceTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
    spdlog::debug("Inference time: {} ms", inferenceTime);
    std::vector<DetectedArmor> acceptedArmors_NG;

    int cnt = 0;
    for (auto const & armor : detectResults) {
        if (armor.color_id == !(params.enemy_color())) { // This is due to inconsistency between model representation and old Solais representation, looks ridiculous but works anyway
            std::cout << "Armor " << cnt << ": " << armor.pts[0] << " " << armor.pts[1] << " " << armor.pts[2] << " " << armor.pts[3] << " " << std::endl;
            std::array<Point2f, 4> armorPoints = armor.pts;

            auto leftVector = armorPoints[1] - armorPoints[0];      // pointing up
            auto rightVector = armorPoints[2] - armorPoints[3];     // pointing up
            auto topVector = armorPoints[2] - armorPoints[1];       // pointing right
            auto bottomVector = armorPoints[3] - armorPoints[0];    // pointing right

            double armorHeight = (cv::norm(leftVector) + cv::norm(rightVector)) / 2;
            double armorWidth = (cv::norm(topVector) + cv::norm(bottomVector)) / 2;

            bool largeArmor;
            if (inRange(armorWidth / armorHeight, params.small_armor_aspect_ratio())) {
                largeArmor = false;
            } else if (inRange(armorWidth / armorHeight, params.large_armor_aspect_ratio())) {
                largeArmor = true;
            } else {
                continue;
            }

            Point2f center = {0, 0};
            for (int i = 0; i < 4; i++) {
                center.x += armorPoints[i].x;
                center.y += armorPoints[i].y;
            }
            // Just use the average X and Y coordinate for the four point
            center.x /= 4;
            center.y /= 4;

            float leftAngle = cv::fastAtan2(leftVector.y, leftVector.x);
            float rightAngle = cv::fastAtan2(rightVector.y, rightVector.x);
            float angleDiff = std::abs(leftAngle - rightAngle);

            acceptedArmors_NG.emplace_back(DetectedArmor{armorPoints,
                                                      center,
                                                      largeArmor,
                                                      0,
                                                      {0 ,0},
                                                      angleDiff,
                                                      (normalizeLightAngle(leftAngle) +\
                                                       normalizeLightAngle(rightAngle)) / 2});
            cnt++;
        }
    }

    return acceptedArmors_NG;
}
#endif
std::vector<ArmorDetector::DetectedArmor>::iterator
ArmorDetector::filterAcceptedArmorsToRemove(std::vector<DetectedArmor> &acceptedArmors) const {
    for (auto it = acceptedArmors.begin(); it != acceptedArmors.end(); ++it) {
        for (auto it2 = it + 1; it2 != acceptedArmors.end(); ++it2) {
            if (it->lightIndex[0] == it2->lightIndex[0] || it->lightIndex[0] == it2->lightIndex[1] ||
                it->lightIndex[1] == it2->lightIndex[0] || it->lightIndex[1] == it2->lightIndex[1]) {
                // Share light

                if (it->largeArmor != it2->largeArmor) {  // one small one large, prioritize small
                    return it->largeArmor ? it : it2;
                }

                // Remove the one that has lights more nonparallel
                return (it->lightAngleDiff > it2->lightAngleDiff) ? it : it2;
            }
        }
    }
    return acceptedArmors.end();  // nothing to remove
}

void ArmorDetector::drawRotatedRect(Mat &img, const RotatedRect &rect, const Scalar &boarderColor) {
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], boarderColor, 2);
    }
}

void ArmorDetector::canonicalizeRotatedRect(cv::RotatedRect &rect) {
    // https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned/21427814#21427814

    if (rect.size.width > rect.size.height) {
        std::swap(rect.size.width, rect.size.height);
        rect.angle += 90;
    }
    if (rect.angle < 0) {
        rect.angle += 180;
    } else if (rect.angle >= 180) {
        rect.angle -= 180;
    }
}

}