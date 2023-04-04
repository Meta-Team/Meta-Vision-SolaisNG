//
// Created by niceme on 3/27/23.
//
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <NvOnnxParser.h>
#include "NanoDet_TensorRT.h"



struct object_rect {
    int x;
    int y;
    int width;
    int height;
};

void resize_uniform(cv::Mat& src, cv::Mat& dst, cv::Size dst_size, object_rect& effect_area)
{
    int w = src.cols;
    int h = src.rows;
    int dst_w = dst_size.width;
    int dst_h = dst_size.height;
    //std::cout << "src: (" << h << ", " << w << ")" << std::endl;
    dst = cv::Mat(cv::Size(dst_w, dst_h), CV_8UC3, cv::Scalar(0));

    double ratio_src = w * 1.0 / h;
    double ratio_dst = dst_w * 1.0 / dst_h;

    int tmp_w = 0;
    int tmp_h = 0;
    if (ratio_src > ratio_dst) {
        tmp_w = dst_w;
        tmp_h = floor((dst_w * 1.0 / w) * h);
    }
    else if (ratio_src < ratio_dst) {
        tmp_h = dst_h;
        tmp_w = floor((dst_h * 1.0 / h) * w);
    }
    else {
        cv::resize(src, dst, dst_size);
        effect_area.x = 0;
        effect_area.y = 0;
        effect_area.width = dst_w;
        effect_area.height = dst_h;
        return;
    }

    //std::cout << "tmp: (" << tmp_h << ", " << tmp_w << ")" << std::endl;
    cv::Mat tmp;
    cv::resize(src, tmp, cv::Size(tmp_w, tmp_h));

    if (tmp_w != dst_w) {
        int index_w = floor((dst_w - tmp_w) / 2.0);
        //std::cout << "index_w: " << index_w << std::endl;
        for (int i = 0; i < dst_h; i++) {
            memcpy(dst.data + i * dst_w * 3 + index_w * 3, tmp.data + i * tmp_w * 3, tmp_w * 3);
        }
        effect_area.x = index_w;
        effect_area.y = 0;
        effect_area.width = tmp_w;
        effect_area.height = tmp_h;
    }
    else if (tmp_h != dst_h) {
        int index_h = floor((dst_h - tmp_h) / 2.0);
        //std::cout << "index_h: " << index_h << std::endl;
        memcpy(dst.data + index_h * dst_w * 3, tmp.data, tmp_w * tmp_h * 3);
        effect_area.x = 0;
        effect_area.y = index_h;
        effect_area.width = tmp_w;
        effect_area.height = tmp_h;
    }
    else {
        printf("error\n");
    }
    //cv::imshow("dst", dst);
    //cv::waitKey(0);
}

const int color_list[80][3] =
        {
                //{255 ,255 ,255}, //bg
                {216 , 82 , 24},
                {236 ,176 , 31},
                {125 , 46 ,141},
                {118 ,171 , 47},
                { 76 ,189 ,237},
                {238 , 19 , 46},
                { 76 , 76 , 76},
                {153 ,153 ,153},
                {255 ,  0 ,  0},
                {255 ,127 ,  0},
                {190 ,190 ,  0},
                {  0 ,255 ,  0},
                {  0 ,  0 ,255},
                {170 ,  0 ,255},
                { 84 , 84 ,  0},
                { 84 ,170 ,  0},
                { 84 ,255 ,  0},
                {170 , 84 ,  0},
                {170 ,170 ,  0},
                {170 ,255 ,  0},
                {255 , 84 ,  0},
                {255 ,170 ,  0},
                {255 ,255 ,  0},
                {  0 , 84 ,127},
                {  0 ,170 ,127},
                {  0 ,255 ,127},
                { 84 ,  0 ,127},
                { 84 , 84 ,127},
                { 84 ,170 ,127},
                { 84 ,255 ,127},
                {170 ,  0 ,127},
                {170 , 84 ,127},
                {170 ,170 ,127},
                {170 ,255 ,127},
                {255 ,  0 ,127},
                {255 , 84 ,127},
                {255 ,170 ,127},
                {255 ,255 ,127},
                {  0 , 84 ,255},
                {  0 ,170 ,255},
                {  0 ,255 ,255},
                { 84 ,  0 ,255},
                { 84 , 84 ,255},
                { 84 ,170 ,255},
                { 84 ,255 ,255},
                {170 ,  0 ,255},
                {170 , 84 ,255},
                {170 ,170 ,255},
                {170 ,255 ,255},
                {255 ,  0 ,255},
                {255 , 84 ,255},
                {255 ,170 ,255},
                { 42 ,  0 ,  0},
                { 84 ,  0 ,  0},
                {127 ,  0 ,  0},
                {170 ,  0 ,  0},
                {212 ,  0 ,  0},
                {255 ,  0 ,  0},
                {  0 , 42 ,  0},
                {  0 , 84 ,  0},
                {  0 ,127 ,  0},
                {  0 ,170 ,  0},
                {  0 ,212 ,  0},
                {  0 ,255 ,  0},
                {  0 ,  0 , 42},
                {  0 ,  0 , 84},
                {  0 ,  0 ,127},
                {  0 ,  0 ,170},
                {  0 ,  0 ,212},
                {  0 ,  0 ,255},
                {  0 ,  0 ,  0},
                { 36 , 36 , 36},
                { 72 , 72 , 72},
                {109 ,109 ,109},
                {145 ,145 ,145},
                {182 ,182 ,182},
                {218 ,218 ,218},
                {  0 ,113 ,188},
                { 80 ,182 ,188},
                {127 ,127 ,  0},
        };


void draw_bboxes(const cv::Mat& bgr, const std::vector<BoxInfo>& bboxes, object_rect effect_roi)
{
    static const char* class_names[] = { "person", "bicycle", "car", "motorcycle", "airplane", "bus",
                                         "train", "truck", "boat", "traffic light", "fire hydrant",
                                         "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                                         "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
                                         "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                                         "skis", "snowboard", "sports ball", "kite", "baseball bat",
                                         "baseball glove", "skateboard", "surfboard", "tennis racket",
                                         "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
                                         "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
                                         "hot dog", "pizza", "donut", "cake", "chair", "couch",
                                         "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
                                         "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
                                         "toaster", "sink", "refrigerator", "book", "clock", "vase",
                                         "scissors", "teddy bear", "hair drier", "toothbrush"
    };

    cv::Mat image = bgr.clone();
    int src_w = image.cols;
    int src_h = image.rows;
    int dst_w = effect_roi.width;
    int dst_h = effect_roi.height;
    float width_ratio = (float)src_w / (float)dst_w;
    float height_ratio = (float)src_h / (float)dst_h;


    for (const auto & bbox : bboxes)
    {
        cv::Scalar color = cv::Scalar(color_list[bbox.label][0], color_list[bbox.label][1], color_list[bbox.label][2]);
        //fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f %.2f\n", bbox.label, bbox.score,
        //    bbox.x1, bbox.y1, bbox.x2, bbox.y2);

        cv::rectangle(image, cv::Rect(cv::Point((bbox.x1 - effect_roi.x) * width_ratio, (bbox.y1 - effect_roi.y) * height_ratio),
                                      cv::Point((bbox.x2 - effect_roi.x) * width_ratio, (bbox.y2 - effect_roi.y) * height_ratio)), color);

        char text[256];
        sprintf(text, "%s %.1f%%", class_names[bbox.label], bbox.score * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

        int x = (bbox.x1 - effect_roi.x) * width_ratio;
        int y = (bbox.y1 - effect_roi.y) * height_ratio - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > image.cols)
            x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      color, -1);

        cv::putText(image, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
    }

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", image);
}


int image_demo(NanoDet_TensorRT& detector, const char* imagepath) {
    std::vector<std::string> filenames;
    cv::glob(imagepath, filenames, false);

    for (const auto& img_name : filenames)
    {
        cv::Mat image = cv::imread(img_name);
        if (image.empty())
        {
            fprintf(stderr, "cv::imread %s failed\n", img_name.c_str());
            return -1;
        }
        object_rect effect_roi{};
        cv::Mat resized_img;
        resize_uniform(image, resized_img, cv::Size(416, 416), effect_roi);
        auto results = detector.detect(resized_img, 0.4, 0.5);
        draw_bboxes(image, results, effect_roi);
        cv::waitKey(0);
    }
    return 0;
}

int webcam_demo(NanoDet_TensorRT& detector, int cam_id)
{
    cv::Mat image;
    cv::VideoCapture cap(cam_id);

    while (true)
    {
        cap >> image;
        object_rect effect_roi{};
        cv::Mat resized_img;
        resize_uniform(image, resized_img, cv::Size(416, 416), effect_roi);
        auto results = detector.detect(resized_img, 0.4, 0.5);
        draw_bboxes(image, results, effect_roi);
        char c=(char)cv::waitKey(0);
        if(c=='q')
            break;
    }
    return 0;
}

int video_demo(NanoDet_TensorRT& detector, const char* path)
{
    cv::Mat image;
    cv::VideoCapture cap(path);
    int frameCounter = 0;
    int tick = 0;
    double elapsedTime;
    double currentFPS;
    double freq = cv::getTickFrequency();

    while (true)
    {
        cap >> image;
        object_rect effect_roi{};
        cv::Mat resized_img;
        auto t_start = std::chrono::high_resolution_clock::now();
        resize_uniform(image, resized_img, cv::Size(416, 416), effect_roi);
        auto t_end = std::chrono::high_resolution_clock::now();
        std::cout << "resize time: " << std::chrono::duration<double, std::milli>(t_end - t_start).count() << " ms" << std::endl;
        auto results = detector.detect(resized_img, 0.4, 0.5);
//        draw_bboxes(image, results, effect_roi);
//        char c=(char)cv::waitKey(1);
//        if(c=='q')
//            break;

        frameCounter++;
        if (frameCounter % 10 == 0) {
            tick = cv::getTickCount() - tick;
            elapsedTime = tick / freq;
            currentFPS = 10 / elapsedTime;
            std::cout << "Frame rate of processing: " << currentFPS << " FPS" << std::endl;
            tick = cv::getTickCount();
        }
    }
    return 0;
}

int benchmark(NanoDet_TensorRT& detector)
{
    int loop_num = 10;
    int warm_up = 4;

    double time_min = DBL_MAX;
    double time_max = -DBL_MAX;
    double time_avg = 0;
    cv::Mat image(416, 416, CV_8UC3, cv::Scalar(1, 1, 1));
//    cv::Mat image(1920, 1080, CV_8UC3, cv::Scalar(1, 1, 1));

    for (int i = 0; i < warm_up + loop_num; i++)
    {
        auto start = std::chrono::steady_clock::now();
        std::vector<BoxInfo> results;
        results = detector.detect(image, 0.4, 0.5);
        auto end = std::chrono::steady_clock::now();
        double time = std::chrono::duration<double, std::milli>(end - start).count();
        if (i >= warm_up)
        {
            time_min = (std::min)(time_min, time);
            time_max = (std::max)(time_max, time);
            time_avg += time;
        }
    }
    time_avg /= loop_num;
    fprintf(stderr, "%20s  min = %7.2f  max = %7.2f  avg = %7.2f\n", "nanodet", time_min, time_max, time_avg);
    return 0;
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "usage: %s [mode] [path]. \n For webcam mode=0, path is cam id; \n For image demo, mode=1, path is image dir; \n For video demo, mode=2, path is video path; \n For benchmark, mode=3, path is dummy.\n", argv[0]);
        return -1;
    }

    int mode = atoi(argv[1]);

    // Load your TensorRT .engine file here
    std::string engine_file_path = "/home/nvidia/nanodet-plus-m_416.engine";
    NanoDet_TensorRT detector;
    if (!detector.loadEngine(engine_file_path))
    {
        fprintf(stderr, "Failed to load TensorRT engine file: %s\n", engine_file_path.c_str());
        return -1;
    }

    if (mode == 0)
    {
        int cam_id = atoi(argv[2]);
        webcam_demo(detector, cam_id);
    }
    else if (mode == 1)
    {
        const char* imagepath = argv[2];
        image_demo(detector, imagepath);
    }
    else if (mode == 2)
    {
        const char* video_path = argv[2];
        video_demo(detector, video_path);
    }
    else if (mode == 3)
    {
        benchmark(detector);
    }
    else
    {
        fprintf(stderr, "error mode\n");
        return -1;
    }

    return 0;
}
