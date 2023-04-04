//
// Created by niceme on 3/27/23.
//

#ifndef META_VISION_SOLAIS_NANODET_TENSORRT_H
#define META_VISION_SOLAIS_NANODET_TENSORRT_H

#include <opencv2/core/core.hpp>
#include <NvInfer.h>
#include <NvInferRuntimeCommon.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>
#include <cassert>
#include <vector>
#include <algorithm>
#include <numeric>
#include <memory>
#include <iostream>
#include <fstream>
#include "TrtCudaUtils.h"
#include "TrtLogger.h"


typedef struct BoxInfo
{
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int label;
} BoxInfo;


class NanoDet_TensorRT {
public:
    NanoDet_TensorRT();
    ~NanoDet_TensorRT();
    std::vector<BoxInfo> detect(cv::Mat& image, float score_threshold, float nms_threshold);
    bool loadEngine(const std::string& engineFilePath);

private:
    void preprocess(cv::Mat& image) const;
    void prepare_buffer();
    void infer();
    void postprocess(float threshold, std::vector<std::vector<BoxInfo>> &results);
    BoxInfo disPred2Bbox(const float*& dfl_det, int label, float score, int x, int y, int stride) const;
    static void nms(std::vector<BoxInfo>& result, float nms_threshold);
    std::vector<int> strides_{ 8, 16, 32, 64 };
    int input_size_ = 416;
    int num_class_ = 80;
    int reg_max_ = 7;
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t stream{};
    float* gpu_buffers[2] = {nullptr, nullptr};
//    float* cpu_input_buffer = nullptr;
//    float* cpu_output_buffer = nullptr;
    float* zero_copy_buffers[2] = {nullptr, nullptr};
    int64_t input_buffer_size = 0;
    int64_t output_buffer_size = 0;
};


#endif //META_VISION_SOLAIS_NANODET_TENSORRT_H
