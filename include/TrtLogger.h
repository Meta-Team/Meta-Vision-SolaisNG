//
// Created by niceme on 3/28/23.
//

#ifndef META_VISION_SOLAIS_TRTLOGGER_H
#define META_VISION_SOLAIS_TRTLOGGER_H

#include <NvInfer.h>
#include <NvInferRuntimeCommon.h>
#include <iostream>

class Logger : public nvinfer1::ILogger
{
public:
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override
    {
        if (severity < nvinfer1::ILogger::Severity::kWARNING)
            std::cout << msg << std::endl;
    }
};

namespace meta
{
    static Logger gLogger;
//    extern LogStreamConsumer gLogVerbose;
//    extern LogStreamConsumer gLogInfo;
//    extern LogStreamConsumer gLogWarning;
//    extern LogStreamConsumer gLogError;
//    extern LogStreamConsumer gLogFatal;

    void setReportableSeverity(Logger::Severity severity);
} // namespace meta


#endif //META_VISION_SOLAIS_TRTLOGGER_H
