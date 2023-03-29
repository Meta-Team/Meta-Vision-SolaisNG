//
// Created by niceme on 3/27/23.
//

#include "NanoDet_TensorRT.h"


inline float fast_exp(float x)
{
    union {
        uint32_t i;
        float f;
    } v{};
    v.i = (1 << 23) * (1.4426950409 * x + 126.93490512f);
    return v.f;
}

inline float sigmoid(float x)
{
    return 1.0f / (1.0f + fast_exp(-x));
}

template<typename Tp>
int activation_function_softmax(const Tp* src, Tp* dst, int length)
{
    const Tp alpha = *std::max_element(src, src + length);
    Tp denominator{ 0 };

    for (int i = 0; i < length; ++i)
    {
        dst[i] = fast_exp(src[i] - alpha);
        denominator += dst[i];
    }

    for (int i = 0; i < length; ++i)
    {
        dst[i] /= denominator;
    }

    return 0;
}


NanoDet_TensorRT::NanoDet_TensorRT() : stream(nullptr)
{
    cudaError_t error_code = cudaStreamCreate(&stream);
    if (error_code != cudaSuccess) {
        std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__;\
        assert(false);
    }
}

NanoDet_TensorRT::~NanoDet_TensorRT()
{
    cudaStreamDestroy(stream);
    cudaFree(gpu_buffers[0]);
    cudaFree(gpu_buffers[1]);
    cudaFreeHost(cpu_input_buffer);
    cudaFreeHost(cpu_output_buffer);
}

bool NanoDet_TensorRT::loadEngine(const std::string& engineFilePath) {
    std::ifstream engineFile(engineFilePath, std::ios::binary);
    if (!engineFile.is_open()) {
        std::cout << "Error opening engine file: " << engineFilePath << std::endl;
        return false;
    }

    size_t size;
    engineFile.seekg(0, std::ifstream::end);
    size = engineFile.tellg();
    engineFile.seekg(0, std::ifstream::beg);
    char *engineData = new char[size];
    assert(engineData);
    engineFile.read(engineData, size);
    engineFile.close();

    runtime.reset(nvinfer1::createInferRuntime(meta::gLogger));
    assert(runtime != nullptr);
    engine.reset(runtime->deserializeCudaEngine(engineData, size));
    assert(engine != nullptr);
    context.reset(engine->createExecutionContext());
    assert(context != nullptr);

    delete[] engineData;

    prepare_buffer();

    return true;
}

void NanoDet_TensorRT::preprocess(const cv::Mat& image) const {
//    std::vector<float> result(1 * 3 * input_size_ * input_size_);
//    float *data = result.data();
    float* data = cpu_input_buffer;
    cv::Mat flt_img;
    flt_img = image.clone();
    cv::resize(flt_img, flt_img, cv::Size(input_size_, input_size_));
    flt_img.convertTo(flt_img, CV_32FC3, 1 / 1.0);
    std::vector<cv::Mat> split_img(3);
    cv::split(flt_img, split_img);
    int channelLength = input_size_ * input_size_;
    const float img_mean[3] = { 103.53f, 116.28f, 123.675f };
    const float img_std[3] = { 0.017429f, 0.017507f, 0.017125f };
    for (int i = 0; i < 3; ++i) {
        split_img[i] = (split_img[i] - img_mean[i]) * img_std[i];
        memcpy(data, split_img[i].data, channelLength * sizeof(float));
        data += channelLength;
    }
}

std::vector<BoxInfo> NanoDet_TensorRT::detect(const cv::Mat& image, float score_threshold, float nms_threshold) {
    // Preprocess
    auto t_start_pre = std::chrono::high_resolution_clock::now();
    preprocess(image);
    auto t_end_pre = std::chrono::high_resolution_clock::now();
    float total_pre = std::chrono::duration<float, std::milli>(t_end_pre - t_start_pre).count();
    std::cout << "detection prepare image take: " << total_pre << " ms." << std::endl;

    // Inference
    auto t_start = std::chrono::high_resolution_clock::now();
    infer();
    auto t_end = std::chrono::high_resolution_clock::now();
    float total_inf = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    std::cout << "detection inference take: " << total_inf << " ms." << std::endl;

    // Postprocess
    auto r_start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<BoxInfo>> results;
    results.resize(this->num_class_);
    postprocess(score_threshold, results);
    std::vector<BoxInfo> dets;
    for (auto & result : results) {
        nms(result, nms_threshold);
        for (auto box : result) {
            dets.push_back(box);
        }
    }
    auto r_end = std::chrono::high_resolution_clock::now();
    float total_res = std::chrono::duration<float, std::milli>(r_end - r_start).count();
    std::cout << "detection postprocess take: " << total_res << " ms." << std::endl;

    return dets;
}

void NanoDet_TensorRT::postprocess(float threshold, std::vector<std::vector<BoxInfo>>& results)
{
    if (cpu_output_buffer == nullptr)
        return;
    int total_idx = 0;
    for (int stage_idx = 0; stage_idx < (int)strides_.size(); stage_idx++)
    {
        int stride = this->strides_[stage_idx];
        int feature_h = ceil(double(this->input_size_) / stride);
        int feature_w = ceil(double(this->input_size_) / stride);
        // cv::Mat debug_heatmap = cv::Mat::zeros(feature_h, feature_w, CV_8UC3);

        for (int idx = total_idx; idx < feature_h * feature_w + total_idx; idx++)
        {
            int row = (idx - total_idx) / feature_w;
            int col = (idx - total_idx) % feature_w;
            float score = -0.0f;
            int cur_label = 0;
            for (int label = 0; label < this->num_class_; label++) {
                float cur_score = cpu_output_buffer[idx * (num_class_ + 4 * (reg_max_ + 1))+ label];
                if (cur_score > score) {
                    score = cur_score;
                    cur_label = label;
                }
            }
            if (score > threshold) {
                //std::cout << "label:" << cur_label << " score:" << score << std::endl;
                const float* bbox_pred = &cpu_output_buffer[idx * (num_class_ + 4 * (reg_max_ + 1)) + num_class_];
                results[cur_label].push_back(this->disPred2Bbox(bbox_pred, cur_label, score, col, row, stride));
                // debug_heatmap.at<cv::Vec3b>(row, col)[0] = 255;
                // cv::imshow("debug", debug_heatmap);
            }
        }
        total_idx += feature_h * feature_w;
    }
}

BoxInfo NanoDet_TensorRT::disPred2Bbox(const float*& dfl_det, int label, float score, int x, int y, int stride) const {
    float ct_x = (x + 0.5) * stride;
    float ct_y = (y + 0.5) * stride;
    std::vector<float> dis_pred;
    dis_pred.resize(4);
    for (int i = 0; i < 4; i++) {
        float dis = 0;
        float* dis_after_sm = new float[this->reg_max_ + 1];
        activation_function_softmax(dfl_det + i * (this->reg_max_ + 1), dis_after_sm, this->reg_max_ + 1);
        for (int j = 0; j < this->reg_max_ + 1; j++) {
            dis += j * dis_after_sm[j];
        }
        dis *= stride;
        //std::cout << "dis:" << dis << std::endl;
        dis_pred[i] = dis;
        delete[] dis_after_sm;
    }
    float xmin = (std::max)(ct_x - dis_pred[0], .0f);
    float ymin = (std::max)(ct_y - dis_pred[1], .0f);
    float xmax = (std::min)(ct_x + dis_pred[2], (float)this->input_size_);
    float ymax = (std::min)(ct_y + dis_pred[3], (float)this->input_size_);

    //std::cout << xmin << "," << ymin << "," << xmax << "," << xmax << "," << std::endl;
    return BoxInfo { xmin, ymin, xmax, ymax, score, label };
}

void NanoDet_TensorRT::nms(std::vector<BoxInfo>& input_boxes, float NMS_THRESH)
{
    std::sort(input_boxes.begin(), input_boxes.end(), [](BoxInfo a, BoxInfo b) { return a.score > b.score; });
    std::vector<float> vArea(input_boxes.size());
    for (int i = 0; i < int(input_boxes.size()); ++i)
    {
        vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1)
                   * (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
    }
    for (int i = 0; i < int(input_boxes.size()); ++i)
    {
        for (int j = i + 1; j < int(input_boxes.size());)
        {
            float xx1 = (std::max)(input_boxes[i].x1, input_boxes[j].x1);
            float yy1 = (std::max)(input_boxes[i].y1, input_boxes[j].y1);
            float xx2 = (std::min)(input_boxes[i].x2, input_boxes[j].x2);
            float yy2 = (std::min)(input_boxes[i].y2, input_boxes[j].y2);
            float w = (std::max)(float(0), xx2 - xx1 + 1);
            float h = (std::max)(float(0), yy2 - yy1 + 1);
            float inter = w * h;
            float ovr = inter / (vArea[i] + vArea[j] - inter);
            if (ovr >= NMS_THRESH)
            {
                input_boxes.erase(input_boxes.begin() + j);
                vArea.erase(vArea.begin() + j);
            }
            else
            {
                j++;
            }
        }
    }
}

void NanoDet_TensorRT::prepare_buffer() {
    auto volume = [](const nvinfer1::Dims& d) -> int64_t {
        return std::accumulate(d.d, d.d + d.nbDims, 1, std::multiplies<>());
    };
    assert(engine->getNbBindings() == 2);

    // Malloc Input Buffer
    nvinfer1::Dims input_dims = engine->getBindingDimensions(0);
    nvinfer1::DataType input_dtype = engine->getBindingDataType(0);
    input_buffer_size = volume(input_dims) * meta::getTrtElementSize(input_dtype);
    cudaMalloc(&gpu_buffers[0], input_buffer_size);

    // Malloc Input Buffer on CPU
    cudaMallocHost(&cpu_input_buffer, input_buffer_size);

    // Malloc Output Buffer
    nvinfer1::Dims output_dims = engine->getBindingDimensions(1);
    nvinfer1::DataType output_dtype = engine->getBindingDataType(1);
    output_buffer_size = volume(output_dims) * meta::getTrtElementSize(output_dtype);
    cudaMalloc(&gpu_buffers[1], input_buffer_size);

    // Malloc Output Buffer on CPU
    cudaMallocHost(&cpu_output_buffer, output_buffer_size);
}

void NanoDet_TensorRT::infer() {
    cudaMemcpyAsync(gpu_buffers[0], cpu_input_buffer, input_buffer_size, cudaMemcpyHostToDevice, stream);
//    cudaStreamSynchronize(stream);
    context->enqueueV2(gpu_buffers, stream, nullptr); // Async execution
//    context->executeV2(gpu_buffers); // Synchroneous execution
    cudaMemcpyAsync(cpu_output_buffer, gpu_buffers[1], output_buffer_size, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
}

