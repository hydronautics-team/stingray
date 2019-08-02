//
// Created by VLADUSHKED on 04.07.19.
//

/**
 * This node:
 * - receives data from STM32 and publishes it
 * - receives byte array from hardware_bridge and send it to STM32 via UART
 */

#include <pluginlib/class_list_macros.h>
#include "../include/uart_driver_nodelet.h"

static const std::string PARAM_DEVICE = "device";

static const std::string PARAM_BAUDRATE = "baudrate";

static const std::string PARAM_DATA_BYTES = "dataBytes";

static const std::string PARAM_PARITY = "parity";

static const std::string PARAM_STOP_BITS = "stopBits";

static const std::string PARITY_NONE = "none";

static const std::string PARITY_EVEN = "even";

static const std::string PARITY_ODD = "odd";

//static const std::string DEFAULT_DEVICE = "/dev/ttyUSB0";
static const std::string DEFAULT_DEVICE = "/dev/ttyS0";

static const int DEFAULT_BAUDRATE = 57600;

static const int DEFAULT_DATA_BYTES = 8;

static const std::string DEFAULT_PARITY = PARITY_NONE;

static const int DEFAULT_STOP_BITS = 1;

// Needed for serial port library
static const int DEFAULT_SERIAL_TIMEOUT = 1000;

void uart_driver::onInit()
{
    NODELET_INFO("Initializing uart_driver_nodelet...");

    /* Initializing node and parameters */
    ros::NodeHandle& nodeHandle = getNodeHandle();

    std::string device;
    nodeHandle.param(PARAM_DEVICE, device, DEFAULT_DEVICE);

    int baudrate;
    nodeHandle.param(PARAM_BAUDRATE, baudrate, DEFAULT_BAUDRATE);

    int dataBytesInt;
    nodeHandle.param(PARAM_DATA_BYTES, dataBytesInt, DEFAULT_DATA_BYTES);
    serial::bytesize_t dataBytes;
    switch (dataBytesInt) {
        case 5:
            dataBytes = serial::bytesize_t::fivebits;
            break;
        case 6:
            dataBytes = serial::bytesize_t::sixbits;
            break;
        case 7:
            dataBytes = serial::bytesize_t::sevenbits;
            break;
        case 8:
            dataBytes = serial::bytesize_t::eightbits;
            break;
        default:
            NODELET_ERROR("Forbidden data bytes size %d, available sizes: 5, 6, 7, 8", dataBytesInt);
            return;
    }

    std::string parityStr;
    nodeHandle.param(PARAM_PARITY, parityStr, DEFAULT_PARITY);
    std::transform(parityStr.begin(), parityStr.end(), parityStr.begin(), ::tolower);
    serial::parity_t parity;
    if (parityStr == PARITY_EVEN)
        parity = serial::parity_t::parity_even;
    else if (parityStr == PARITY_ODD)
        parity = serial::parity_t::parity_odd;
    else if (parityStr == PARITY_NONE)
        parity = serial::parity_t::parity_none;
    else {
        NODELET_ERROR("Unrecognised parity \"%s\", available parities: \"none\", \"odd\", \"even\"",
                      parityStr.c_str());
        return;
    }

    int stopBitsInt;
    nodeHandle.param(PARAM_STOP_BITS, stopBitsInt, DEFAULT_STOP_BITS);
    serial::stopbits_t stopBits;
    switch (stopBitsInt) {
        case 1:
            stopBits = serial::stopbits_t::stopbits_one;
            break;
        case 2:
            stopBits = serial::stopbits_t::stopbits_two;
            break;
        default:
            NODELET_ERROR("Forbidden stop bits size %d, available sizes: 1, 2", stopBitsInt);
            return;
    }

    // ROS publishers
    NODELET_DEBUG("Initializing publisher /hard_bridge/uart");
    outputMessage_pub = nodeHandle.advertise<std_msgs::UInt8MultiArray>("/hard_bridge/uart",
                                                                        1000);
    // **************

    // ROS subscribers
    NODELET_DEBUG("Initializing subscriber /hard_bridge/parcel");
    inputMessage_sub = nodeHandle.subscribe("/hard_bridge/parcel", 1000,
                                            &uart_driver::inputMessage_callback, this);
    // **************
    /**
     * Initialasing serial port
     * Closes port if it is closed, initialized it
     * with given parameter and DOES NOT OPEN IT.
     */
    NODELET_DEBUG("UART settings: Device: %s, Baudrate: %d, Data bytes: %d, Parity: %s, Stop bits: %d",
                 device.c_str(), baudrate, dataBytes, parityStr.c_str(), stopBitsInt);
    if (port.isOpen()) port.close();

    port.setPort(device);
    serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(DEFAULT_SERIAL_TIMEOUT);
    port.setTimeout(serialTimeout);
    port.setBaudrate(baudrate);
    port.setBytesize(dataBytes);
    port.setParity(parity);
    port.setStopbits(stopBits);

    // Input message container
    NODELET_DEBUG("Initializing uart_driver input message container");
    msg_in.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_in.layout.dim[0].size = RequestMessage::length;
    msg_in.layout.dim[0].stride = RequestMessage::length;
    msg_in.layout.dim[0].label = "msg_in";
    msg_in.data = { 0 };

    // Outnput message container
    NODELET_DEBUG("Initializing uart_driver output message container");
    msg_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_out.layout.dim[0].size = ResponseMessage::length;
    msg_out.layout.dim[0].stride = ResponseMessage::length;
    msg_out.layout.dim[0].label = "msg_out";
    msg_out.data = { 0 };
}

bool uart_driver::sendData()
{
    std::vector<uint8_t> msg;
    for(int i=0; i<RequestMessage::length; i++)
        msg.push_back(msg_in.data[i]);
    size_t toWrite = sizeof(uint8_t) * msg.size();
    try {
        port.flush();
        size_t written = port.write(msg);
        return written == toWrite;
        NODELET_DEBUG("SENDING TO STM");
    }
    catch (serial::IOException &ex){
        NODELET_ERROR("Serial exception when trying to flush and send. Error: %s", ex.what());
        return false;
    }
}

bool uart_driver::receiveData()
{
    if(port.available() < ResponseMessage::length) {
        return false;
    }

    std::vector<uint8_t> answer;
    port.read(answer, ResponseMessage::length);

    msg_out.data.clear();
    for(int i=0; i<ResponseMessage::length; i++) {
        msg_out.data.push_back(answer[i]);
    }
    NODELET_DEBUG("RECEIVE FROM STM");

    return true;
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
void uart_driver::inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg)
{
    NODELET_DEBUG("uart_driver message callback");
    msg_in.data.clear();
    for(int i = 0; i < RequestMessage::length; i++)
        msg_in.data.push_back(msg->data[i]);
    try {
        if (!port.isOpen()) {
            port.open();
            if (!port.isOpen())
                NODELET_ERROR("Unable to open UART port");
        }
    }
    catch (serial::IOException &ex){
        NODELET_ERROR("Serial exception when trying to open. Error: %s", ex.what());
        return;
    }
    if(!sendData()) {
        NODELET_ERROR("Unable to send msg to STM32");
        return;
    }
    if(receiveData())
        outputMessage_pub.publish(msg_out);
    else {
        NODELET_ERROR("Unable to receive msg from STM32");
        return;
    }
}

PLUGINLIB_EXPORT_CLASS(uart_driver, nodelet::Nodelet);
