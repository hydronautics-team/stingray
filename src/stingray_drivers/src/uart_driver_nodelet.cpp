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

void uart_driver::onInit() {
    // Initializing nodelet and parameters
    NODELET_INFO_STREAM("Initializing nodelet: " << UART_DRIVER_NODE_NAME);
    ros::NodeHandle& nodeHandle = getNodeHandle();
    //Serial port initialization
    portInitialize(nodeHandle);
    // ROS publishers
    NODELET_DEBUG_STREAM("Initializing publisher" << INPUT_PARCEL_TOPIC);
    outputMessage_pub = nodeHandle.advertise<std_msgs::UInt8MultiArray>(INPUT_PARCEL_TOPIC,1000);
    // ROS subscribers
    NODELET_DEBUG_STREAM("Initializing subscriber" << OUTPUT_PARCEL_TOPIC);
    inputMessage_sub = nodeHandle.subscribe(OUTPUT_PARCEL_TOPIC, 1000,
            &uart_driver::inputMessage_callback, this);
    // Input message container
    NODELET_DEBUG_STREAM("Initializing " << UART_DRIVER_NODE_NAME << " input message container");
    inputMessage.layout.dim.push_back(std_msgs::MultiArrayDimension());
    inputMessage.layout.dim[0].size = RequestMessage::length;
    inputMessage.layout.dim[0].stride = RequestMessage::length;
    inputMessage.layout.dim[0].label = "inputMessage";
    inputMessage.data = {0};
    // Outnput message container
    NODELET_DEBUG_STREAM("Initializing " << UART_DRIVER_NODE_NAME << " output message container");
    outputMessage.layout.dim.push_back(std_msgs::MultiArrayDimension());
    outputMessage.layout.dim[0].size = ResponseMessage::length;
    outputMessage.layout.dim[0].stride = ResponseMessage::length;
    outputMessage.layout.dim[0].label = "outputMessage";
    outputMessage.data = {0};
}
/**
 * Initialasing serial port
 * Closes port if it is closed, initialized it
 * with given parameter and DOES NOT OPEN IT.
 */
void uart_driver::portInitialize(ros::NodeHandle& nodeHandle) {
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
}

bool uart_driver::sendData() {
    std::vector<uint8_t> msg;
    for(int i=0; i<RequestMessage::length; i++)
        msg.push_back(inputMessage.data[i]);
    size_t toWrite = sizeof(uint8_t) * msg.size();
    try {
        port.flush();
        size_t written = port.write(msg);
        return written == toWrite;
    }
    catch (serial::IOException &ex){
        NODELET_ERROR("Serial exception, when trying to flush and send. Error: %s", ex.what());
        return false;
    }
}

bool uart_driver::receiveData() {
    if(port.available() < ResponseMessage::length)
        return false;
    std::vector<uint8_t> answer;
    port.read(answer, ResponseMessage::length);
    outputMessage.data.clear();
    for(int i=0; i<ResponseMessage::length; i++)
        outputMessage.data.push_back(answer[i]);
    NODELET_DEBUG("RECEIVE FROM STM");

    return true;
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
void uart_driver::inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg) {
    NODELET_DEBUG_STREAM(UART_DRIVER_NODE_NAME << " message callback");
    inputMessage.data.clear();
    for(int i = 0; i < RequestMessage::length; i++)
        inputMessage.data.push_back(msg->data[i]);
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
        NODELET_ERROR("Unable to send message to STM32");
        return;
    }
    if(receiveData())
        outputMessage_pub.publish(outputMessage);
    else {
        NODELET_ERROR("Unable to receive message from STM32");
        return;
    }
}
PLUGINLIB_EXPORT_CLASS(uart_driver, nodelet::Nodelet);
