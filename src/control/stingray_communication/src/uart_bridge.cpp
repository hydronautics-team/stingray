/**
 * This node:
 * - receives data from STM32 and publishes it
 * - receives byte array from hardware_bridge and send it to STM32 via UART
 */

#include "../include/uart_bridge.h"

static const std::string UART_BRIDGE_NODE_NAME = "uart_bridge";
static const std::string PARAM_DEVICE = "device";
static const std::string PARAM_BAUDRATE = "baudrate";
static const std::string PARAM_DATA_BYTES = "dataBytes";
static const std::string PARAM_PARITY = "parity";
static const std::string PARAM_STOP_BITS = "stopBits";
static const std::string PARITY_NONE = "none";
static const std::string PARITY_EVEN = "even";
static const std::string PARITY_ODD = "odd";
static const std::string DEFAULT_DEVICE = "/dev/ttyS0";
static const int DEFAULT_BAUDRATE = 57600;
static const int DEFAULT_DATA_BYTES = 8;
static const std::string DEFAULT_PARITY = PARITY_NONE;
static const int DEFAULT_STOP_BITS = 1;
static const int DEFAULT_SERIAL_TIMEOUT = 1000; // Needed for serial port library

void UartBridge::UartBridge() : Node(UART_BRIDGE_NODE_NAME) {
    //Serial port initialization
    portInitialize(nodeHandle);
    // ROS publishers
    outputMessage_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(INPUT_PARCEL_TOPIC, 1000);
    // ROS subscribers
    inputMessage_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(OUTPUT_PARCEL_TOPIC, 1000,
                                                                                 std::bind(
                                                                                         &HardwareBridge::inputMessage_callback,
                                                                                         this, _1));
    // Input message container
    inputMessage.layout.dim.push_back(std_msgs::MultiArrayDimension());
    inputMessage.layout.dim[0].size = RequestMessage::length;
    inputMessage.layout.dim[0].stride = RequestMessage::length;
    inputMessage.layout.dim[0].label = "inputMessage";
    inputMessage.data = {0};
    // Outnput message container
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
void UartBridge::portInitialize(ros::NodeHandle &nodeHandle) {
    std::string device;
    this->declare_parameter<std::string>(PARAM_DEVICE, DEFAULT_DEVICE);
    this->get_parameter(PARAM_DEVICE, device);
    int baudrate;
    this->declare_parameter<std::string>(PARAM_BAUDRATE, DEFAULT_BAUDRATE);
    this->get_parameter(PARAM_BAUDRATE, baudrate);
    int dataBytesInt;
    this->declare_parameter<std::string>(PARAM_DATA_BYTES, DEFAULT_DATA_BYTES);
    this->get_parameter(PARAM_DATA_BYTES, dataBytesInt);
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
            RCLCPP_ERROR(this->get_logger(), "Forbidden data bytes size %d, available sizes: 5, 6, 7, 8", dataBytesInt);
            return;
    }
    std::string parityStr;
    this->declare_parameter<std::string>(PARAM_PARITY, DEFAULT_PARITY);
    this->get_parameter(PARAM_PARITY, parityStr);
    std::transform(parityStr.begin(), parityStr.end(), parityStr.begin(), ::tolower);
    serial::parity_t parity;
    if (parityStr == PARITY_EVEN)
        parity = serial::parity_t::parity_even;
    else if (parityStr == PARITY_ODD)
        parity = serial::parity_t::parity_odd;
    else if (parityStr == PARITY_NONE)
        parity = serial::parity_t::parity_none;
    else {
        RCLCPP_ERROR(this->get_logger(), "Unrecognised parity \"%s\", available parities: \"none\", \"odd\", \"even\"",
                     parityStr.c_str());
        return;
    }
    int stopBitsInt;
    this->declare_parameter<std::string>(PARAM_STOP_BITS, DEFAULT_STOP_BITS);
    this->get_parameter(PARAM_STOP_BITS, stopBitsInt);
    serial::stopbits_t stopBits;
    switch (stopBitsInt) {
        case 1:
            stopBits = serial::stopbits_t::stopbits_one;
            break;
        case 2:
            stopBits = serial::stopbits_t::stopbits_two;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Forbidden stop bits size %d, available sizes: 1, 2", stopBitsInt);
            return;
    }
    RCLCPP_INFO(this->get_logger(),
                "UART settings: Device: %s, Baudrate: %d, Data bytes: %d, Parity: %s, Stop bits: %d",
                device.c_str(), baudrate, dataBytes, parityStr.c_str(), stopBitsInt);
    if (port.isOpen())
        port.close();
    port.setPort(device);
    serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(DEFAULT_SERIAL_TIMEOUT);
    port.setTimeout(serialTimeout);
    port.setBaudrate(baudrate);
    port.setBytesize(dataBytes);
    port.setParity(parity);
    port.setStopbits(stopBits);
}

bool UartBridge::sendData() {
    std::vector <uint8_t> msg;
    for (int i = 0; i < RequestMessage::length; i++)
        msg.push_back(inputMessage.data[i]);
    size_t toWrite = sizeof(uint8_t) * msg.size();
    try {
        port.flush();
        size_t written = port.write(msg);
        return written == toWrite;
    }
    catch (serial::IOException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Serial exception, when trying to flush and send. Error: %s", ex.what());
        return false;
    }
}

bool UartBridge::receiveData() {
    if (port.available() < ResponseMessage::length)
        return false;
    std::vector <uint8_t> answer;
    port.read(answer, ResponseMessage::length);
    outputMessage.data.clear();
    for (int i = 0; i < ResponseMessage::length; i++)
        outputMessage.data.push_back(answer[i]);
    RCLCPP_DEBUG(this->get_logger(), "RECEIVE FROM STM");

    return true;
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
void UartBridge::inputMessage_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) const {
    RCLCPP_DEBUG(this->get_logger(), UART_BRIDGE_NODE_NAME + " message callback");
    inputMessage.data.clear();
    for (int i = 0; i < RequestMessage::length; i++)
        inputMessage.data.push_back(msg->data[i]);
    try {
        if (!port.isOpen()) {
            port.open();
            if (!port.isOpen())
                RCLCPP_ERROR(this->get_logger(), "Unable to open UART port");
        }
    }
    catch (serial::IOException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Serial exception when trying to open. Error: %s", ex.what());
        return;
    }
    if (!sendData()) {
        RCLCPP_ERROR(this->get_logger(), "Unable to send message to STM32");
        return;
    }
    if (receiveData())
        outputMessage_pub.publish(outputMessage);
    else {
        RCLCPP_ERROR(this->get_logger(), "Unable to receive message from STM32");
        return;
    }
}
