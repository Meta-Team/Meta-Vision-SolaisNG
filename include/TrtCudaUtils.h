/*
 * SPDX-FileCopyrightText: Copyright (c) 1993-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef META_VISION_SOLAIS_TRTCUDAUTILS_H
#define META_VISION_SOLAIS_TRTCUDAUTILS_H


#include <cassert>
#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include <thread>
#include "TrtLogger.h"

namespace meta
{

    inline void cudaCheck(cudaError_t ret, std::ostream& err = std::cerr)
    {
        if (ret != cudaSuccess)
        {
            err << "Cuda failure: " << cudaGetErrorString(ret) << std::endl;
            abort();
        }
    }

    inline void setCudaDevice(int device, std::ostream& os)
    {
        cudaCheck(cudaSetDevice(device));

        cudaDeviceProp properties{};
        cudaCheck(cudaGetDeviceProperties(&properties, device));

        // clang-format off
        os << "=== Device Information ===" << std::endl;
        os << "Selected Device: "      << properties.name                                               << std::endl;
        os << "Compute Capability: "   << properties.major << "." << properties.minor                   << std::endl;
        os << "SMs: "                  << properties.multiProcessorCount                                << std::endl;
        os << "Compute Clock Rate: "   << properties.clockRate / 1000000.0F << " GHz"                   << std::endl;
        os << "Device Global Memory: " << (properties.totalGlobalMem >> 20) << " MiB"                   << std::endl;
        os << "Shared Memory per SM: " << (properties.sharedMemPerMultiprocessor >> 10) << " KiB"       << std::endl;
        os << "Memory Bus Width: "     << properties.memoryBusWidth << " bits"
           << " (ECC " << (properties.ECCEnabled != 0 ? "enabled" : "disabled") << ")" << std::endl;
        os << "Memory Clock Rate: "    << properties.memoryClockRate / 1000000.0F << " GHz"             << std::endl;
        // clang-format on
    }

    inline int32_t getCudaDriverVersion()
    {
        int32_t version{-1};
        cudaCheck(cudaDriverGetVersion(&version));
        return version;
    }

    inline int32_t getCudaRuntimeVersion()
    {
        int32_t version{-1};
        cudaCheck(cudaRuntimeGetVersion(&version));
        return version;
    }

    inline unsigned int getTrtElementSize(nvinfer1::DataType t)
    {
        switch (t)
        {
            case nvinfer1::DataType::kINT32:
            case nvinfer1::DataType::kFLOAT: return 4;
            case nvinfer1::DataType::kHALF: return 2;
            case nvinfer1::DataType::kBOOL:
            case nvinfer1::DataType::kUINT8:
            case nvinfer1::DataType::kINT8: return 1;
        }
        throw std::runtime_error("Invalid DataType.");
    }

} // namespace meta

#endif //META_VISION_SOLAIS_TRTCUDAUTILS_H
