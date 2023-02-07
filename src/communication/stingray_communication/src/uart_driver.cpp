/**
 * This node:
 * - receives data from STM32 and publishes it
 * - receives byte array from hardware_bridge and send it to STM32 via UART
 */

#include <fstream>
#include "uart_driver.h"

UartDriver::UartDriver() : Node("UartDriver")
{
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    hardware_config = json::parse(std::ifstream("resources/configs/hardware.json"));
    // Serial port initialization
    portInitialize();
    // ROS publishers
    this->outputMessage_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["input_parcel"], 1000);
    // ROS subscribers
    this->inputMessage_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["output_parcel"], 1000,
                                                                                       std::bind(
                                                                                           &UartDriver::inputMessage_callback,
                                                                                           this, _1));
    // Input message container
    inputMessage.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    inputMessage.layout.dim[0].size = RequestMessage::length;
    inputMessage.layout.dim[0].stride = RequestMessage::length;
    inputMessage.layout.dim[0].label = "inputMessage";
    inputMessage.data = {0};
    // Outnput message container
    outputMessage.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
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
void UartDriver::portInitialize()
{
    std::string device = hardware_config["uart"]["device"];
    int baudrate = hardware_config["uart"]["baudrate"];
    int dataBytesInt = hardware_config["uart"]["data_bytes"];
    serial::bytesize_t dataBytes;
    switch (dataBytesInt)
    {
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
    std::string parityStr = hardware_config["uart"]["parity"];
    std::transform(parityStr.begin(), parityStr.end(), parityStr.begin(), ::tolower);
    serial::parity_t parity;
    if (parityStr == "even")
        parity = serial::parity_t::parity_even;
    else if (parityStr == "odd")
        parity = serial::parity_t::parity_odd;
    else if (parityStr == "none")
        parity = serial::parity_t::parity_none;
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unrecognised parity \"%s\", available parities: \"none\", \"odd\", \"even\"",
                     parityStr.c_str());
        return;
    }
    int stopBitsInt = hardware_config["uart"]["stop_bits"];
    serial::stopbits_t stopBits;
    switch (stopBitsInt)
    {
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
    serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(hardware_config["uart"]["serial_timeout"]);
    port.setTimeout(serialTimeout);
    port.setBaudrate(baudrate);
    port.setBytesize(dataBytes);
    port.setParity(parity);
    port.setStopbits(stopBits);
}

bool UartDriver::sendData()
{
    std::vector<uint8_t> msg;
    for (int i = 0; i < RequestMessage::length; i++)
        msg.push_back(inputMessage.data[i]);
    size_t toWrite = sizeof(uint8_t) * msg.size();
    try
    {
        port.flush();
        size_t written = port.write(msg);
        return written == toWrite;
    }
    catch (serial::IOException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial exception, when trying to flush and send. Error: %s", ex.what());
        return false;
    }
}

bool UartDriver::receiveData()
{
    if (port.available() < ResponseMessage::length)
        return false;
    std::vector<uint8_t> answer;
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
void UartDriver::inputMessage_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    inputMessage.data.clear();
    for (int i = 0; i < RequestMessage::length; i++)
        inputMessage.data.push_back(msg->data[i]);
    try
    {
        if (!port.isOpen())
        {
            port.open();
            if (!port.isOpen())
                RCLCPP_ERROR(this->get_logger(), "Unable to open UART port");
        }
    }
    catch (serial::IOException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial exception when trying to open. Error: %s", ex.what());
        return;
    }
    if (!sendData())
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send message to STM32");
        return;
    }
    if (receiveData())
        outputMessage_pub->publish(outputMessage);
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to receive message from STM32");
        return;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<UartDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
