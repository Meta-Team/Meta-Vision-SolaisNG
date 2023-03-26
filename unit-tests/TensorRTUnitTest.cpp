//
// Created by niceme on 3/19/23.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <chrono>
//#include <cmath>

#include "logging.h"
#include "cuda_utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "utils.h"
#include "model.h"

using namespace nvinfer1;

//const std::string classNamesFile = "./nn-models/coco.names";
const std::string enginePath = "/home/niceme/Meta-Vision-SolaisNG/nn-models/rm_yolov5s.engine";
static Logger gLogger;
const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;


std::vector<std::string> readClassNames(const std::string& filename) {
    std::vector<std::string> classNames;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open class names file: " << filename << std::endl;
        return classNames;
    }

    std::string line;
    while (std::getline(file, line)) {
        classNames.push_back(line);
    }

    return classNames;
}

void deserialize_engine(const std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size;
    file.seekg(0, std::ifstream::end);
    size = file.tellg();
    file.seekg(0, std::ifstream::beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer, float** cpu_output_buffer) {
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)))
    CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)))

    *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output, int batchsize) {
    context.enqueue(batchsize, gpu_buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream))
    cudaStreamSynchronize(stream);
}

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <video file>" << std::endl;
        return -1;
    }

    std::string videoPath = argv[1];
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video file: " << videoPath << std::endl;
        return -1;
    }

//    std::vector<std::string> classNames = readClassNames(classNamesFile);

    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    deserialize_engine(enginePath, &runtime, &engine, &context);
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream))

    cuda_preprocess_init(kMaxInputImageSize);

    float* gpu_buffers[2];
    float* cpu_output_buffer = nullptr;
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

    int frameWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frameHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);

    // Prepare Output Video
//    cv::VideoWriter outputVideo("Output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frameWidth, frameHeight));
//
//    if (!outputVideo.isOpened()) {
//        std::cout << "Error opening the output video file." << std::endl;
//        return -1;
//    }

    fps = cap.get(cv::CAP_PROP_FPS);
    namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
    int frameCounter = 0;
    int tick = 0;
    double elapsedTime;
    double currentFPS;
    double freq = cv::getTickFrequency();

    while (true) {

        // Read a frame
        cv::Mat frame;
        cap >> frame;

        // Break the loop if the end of the video is reached
        if (frame.empty()) break;

        // Get a batch of images
        std::vector<cv::Mat> img_batch;
        std::vector<std::string> img_name_batch;
        img_batch.push_back(frame);

        // Preprocess
        cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

        // Run inference
        auto start = std::chrono::system_clock::now();
        infer(*context, stream, (void **) gpu_buffers, cpu_output_buffer, kBatchSize);
        auto end = std::chrono::system_clock::now();
//        std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//                  << "ms" << std::endl;

        // NMS
        std::vector<std::vector<Detection>> res_batch;
        batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

        // Draw bounding boxes
        draw_bbox(img_batch, res_batch);

        // Display Video
        cv::imshow("Detection Result", img_batch[0]);
        char c=(char)cv::waitKey(1);
        if(c=='q')
            break;

        frameCounter++;
        if (frameCounter % 10 == 0) {
            tick = cv::getTickCount() - tick;
            elapsedTime = tick / freq;
            currentFPS = 10 / elapsedTime;
            std::cout << "Frame rate of processing: " << currentFPS << " FPS" << std::endl;
            tick = cv::getTickCount();
        }

        // Write into Video
//        outputVideo << img_batch[0];
    }

    cap.release();
//    outputVideo.release();
    cv::destroyAllWindows();


    cudaStreamDestroy(stream);
    cudaFree(gpu_buffers[0]);
    cudaFree(gpu_buffers[1]);
    delete[] cpu_output_buffer;
    cuda_preprocess_destroy();

    delete context;
    delete engine;
    delete runtime;

    return 0;
}