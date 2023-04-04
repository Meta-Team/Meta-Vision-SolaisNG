//
// Created by niceme on 2023/3/30.
//

#include <iostream>



__global__ void preprocess_kernel(float *data, const float *image_data, int input_size, int channelLength, const float *img_mean, const float *img_std) {
//    unsigned int c = blockIdx.x;
    unsigned int idx = threadIdx.x + blockDim.x * blockIdx.y;

    if (idx < channelLength) {
        int row = idx / input_size;
        int col = idx % input_size;
//        int offset = c * channelLength + row * input_size + col;
        int offset = idx;

        float pixel = image_data[offset];
        data[offset] = (pixel - img_mean[0]) * img_std[0];
    }
}


int main() {
    std::string engine_file_path = "/home/nvidia/nanodet-plus-m_416.engine";

    return 0;
}