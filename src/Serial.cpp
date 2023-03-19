//
// Created by liuzikai on 3/28/21.
//

#include "Serial.h"
#include "CRC.h"
#include <iostream>
namespace meta {

Serial::Serial(boost::asio::io_context &ioContext)
        : ioContext(ioContext), serial(ioContext) {

    boost::system::error_code ec;
    // SERIAL_DEVICE defined in CMakeLists.txt
    serial.open(SERIAL_DEVICE, ec);
    if (ec) {
        std::cerr << "Failed to open serial device \"" << SERIAL_DEVICE << "\": " << ec.message() << std::endl;
        std::exit(1);
    }
#ifndef BOOST_OS_WINDOWS
    ::tcflush(serial.lowest_layer().native_handle(), TCIOFLUSH);  // flush input and output
#endif
    serial.set_option(boost::asio::serial_port::baud_rate(SERIAL_BAUD_RATE));
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));

    // Start receiving SOF
    boost::asio::async_read(serial,
                            boost::asio::buffer(((uint8_t *) &recvPackage), 1),
                            boost::asio::transfer_exactly(1),
                            [this](auto &error, auto numBytes) { handleRecv(error, numBytes); });
}

bool Serial::sendControlCommand(bool detected, bool topKillerTriggered, TimePoint time, float yawDelta, float pitchDelta, float distance,
                                float avgLightAngle, float imageX, float imageY, int remainingTimeToTarget, int period) {

    auto pkg = std::make_shared<Package>();

    pkg->sof = SOF;
    pkg->cmdID = VISION_CONTROL_CMD_ID;

    pkg->command.flag = 0;
    if (detected) pkg->command.flag |= VisionCommand::DETECTED;
    if (topKillerTriggered) pkg->command.flag |= VisionCommand::TOP_KILLER_TRIGGERED;
    pkg->command.frameTime = time;
    pkg->command.yawDelta = (int16_t) (-yawDelta * 100);  // notice the minus sign
    pkg->command.pitchDelta = (int16_t) (pitchDelta * 100);
    pkg->command.distance = (int16_t) distance;
    pkg->command.avgLightAngle = (int16_t) (avgLightAngle * 100);
    pkg->command.imageX = (int16_t) imageX;
    pkg->command.imageY = (int16_t) imageY;
    pkg->command.remainingTimeToTarget = (int16_t) remainingTimeToTarget;
    pkg->command.period = (int16_t) period;
    rm::appendCRC8CheckSum((uint8_t *) (pkg.get()), sizeof(uint8_t) * 2 + sizeof(VisionCommand) + sizeof(uint8_t));

    //::tcflush(serial.lowest_layer().native_handle(), TCIFLUSH);  // clear input buffer
    boost::asio::async_write(
            serial,
            boost::asio::buffer(pkg.get(), sizeof(uint8_t) * 2 + sizeof(VisionCommand) + sizeof(uint8_t)),
            [this, pkg](auto &error, auto numBytes) { handleSend(pkg, error, numBytes); }
    );

    return true;
}

void Serial::handleSend(std::shared_ptr<Package> buf, const boost::system::error_code &error,
                        size_t numBytes) {
    if (error) {
        std::cerr << "Serial: send error: " << error.message() << "\n";
    }
    ++cumulativeFrameCounter;
}

void Serial::handleRecv(const boost::system::error_code &error, size_t numBytes) {

    if (error) {
        std::cerr << "Serial: recv error: " << error.message() << "\n";
        // Continue to process data and start next async_recv
    }

    size_t nextOffset, nextSize;

    switch (recvState) {

        case RECV_PREAMBLE:
            if (recvPackage.sof == SOF) {
                recvState = RECV_CMD_ID;
            } // otherwise, keep waiting for SOF
            break;

        case RECV_CMD_ID:
            if (recvPackage.cmdID < CMD_ID_COUNT) {
                recvState = RECV_DATA_TAIL;
            } else {
                recvState = RECV_PREAMBLE;
            }
            break;

        case RECV_DATA_TAIL:
            if (rm::verifyCRC8CheckSum((uint8_t *) &recvPackage,
                                       sizeof(uint8_t) * 2 + DATA_SIZE[recvPackage.cmdID] + sizeof(uint8_t))) {

                switch (recvPackage.cmdID) {

                }
            }
            recvState = RECV_PREAMBLE;
            break;
    }


    switch (recvState) {
        case RECV_PREAMBLE:
            nextOffset = 0;
            nextSize = sizeof(uint8_t);
            break;
        case RECV_CMD_ID:
            nextOffset = sizeof(uint8_t);
            nextSize = sizeof(uint8_t);
            break;
        case RECV_DATA_TAIL:
            nextOffset = sizeof(uint8_t) * 2;
            nextSize = DATA_SIZE[recvPackage.cmdID] + sizeof(uint8_t);
            break;
    }

    boost::asio::async_read(serial,
                            boost::asio::buffer(((uint8_t *) &recvPackage) + nextOffset, nextSize),
                            boost::asio::transfer_exactly(nextSize),
                            [this](auto &error, auto numBytes) { handleRecv(error, numBytes); });
}

}