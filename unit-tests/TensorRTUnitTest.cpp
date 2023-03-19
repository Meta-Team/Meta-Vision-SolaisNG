//
// Created by niceme on 3/19/23.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <fstream>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

const std::string modelPath = "/home/niceme/Meta-Vision-SolaisNG/nn-models/yolov3-10.onnx";
const std::string classNamesFile = "./nn-models/coco.names";
const float confidenceThreshold = 0.5;
const float nmsThreshold = 0.4;

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

    // Create a model using the API directly and serialize it to a stream
    char *trtModelStream{nullptr};
    size_t size{0};
    std::ifstream file(modelPath, std::ios::binary);
    if (file.good()) {
        file.seekg(0, std::ifstream::end);
        size = file.tellg();
        file.seekg(0, std::ifstream::beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }

    std::vector<std::string> classNames = readClassNames(classNamesFile);

//    nvinfer1::IRuntime* runtime = createInferRuntime(gLogger);
//    assert(runtime != nullptr);
//    nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size, nullptr);
//    assert(engine != nullptr);
//    nvinfer1::IExecutionContext* context = engine->createExecutionContext();
//    assert(context != nullptr);

    cv::Mat frame;
    while (cap.read(frame)) {
        // todo: run inference
    }
    return 0;
}