//
// Created by niceme on 2023/4/2.
//

#ifndef META_VISION_SOLAIS_YOLOV5_TENSORRT_H
#define META_VISION_SOLAIS_YOLOV5_TENSORRT_H

#include <opencv2/core.hpp>
#include <NvInfer.h>



namespace meta {
    class YOLODet {
        struct alignas(4) bbox_t {
            cv::Point2f pts[4]; // [pt0, pt1, pt2, pt3]
            float confidence;
            int color_id; // 0: blue, 1: red, 2: gray
            int tag_id;   // 0: guard, 1-5: number, 6: base

            bool operator==(const bbox_t&) const = default;
            bool operator!=(const bbox_t&) const = default;
        };

        static constexpr int TOPK_NUM = 128;
        static constexpr float KEEP_THRES = 0.1f;

    public:
        explicit YOLODet(const std::string &onnx_file);

        ~YOLODet();

        YOLODet(const YOLODet &) = delete;

        YOLODet operator=(const YOLODet &) = delete;

        std::vector<bbox_t> operator()(const cv::Mat &src) const;

    private:
        void build_engine_from_onnx(const std::string &onnx_file);

        void build_engine_from_cache(const std::string &cache_file);

        void cache_engine(const std::string &cache_file);

        nvinfer1::ICudaEngine *engine;
        nvinfer1::IExecutionContext *context;
        mutable void *device_buffer[2];
        float *output_buffer;
        cudaStream_t stream;
        int input_idx, output_idx;
        size_t input_sz, output_sz;
    };

} // meta

#endif //META_VISION_SOLAIS_YOLOV5_TENSORRT_H
