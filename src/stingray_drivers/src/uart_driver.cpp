/**
 * This node:
 * - receives data from STM32 and publishes it
 * - receives byte array from hardware_bridge and send it to STM32 via UART
 */

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <serial/serial.h>

#include <sstream>
#include <string>
#include <vector>

#include "messages/messages.h"
#include "TopicsAndServices.h"

static const std::string UART_BRIDGE_NODE_NAME = "uart_bridge";


static const std::string PARAM_DEVICE = "device";
static const std::string PARAM_BAUDRATE = "baudrate";
static const std::string PARAM_DATA_BYTES = "data_bytes";
static const std::string PARAM_PARITY = "parity";
static const std::string PARAM_STOP_BITS = "stop_bits";
static const std::string PARITY_NONE = "none";
static const std::string PARITY_EVEN = "even";
static const std::string PARITY_ODD = "odd";

static const std::string DEFAULT_DEVICE = "/dev/ttyS0";
static const int DEFAULT_BAUDRATE = 57600;
static const int DEFAULT_DATA_BYTES = 8;
static const std::string DEFAULT_PARITY = PARITY_NONE;
static const int DEFAULT_STOP_BITS = 1;

// Needed for serial port library
static const int DEFAULT_SERIAL_TIMEOUT = 1000;

// Hardware bridge -> Protocol_bridge
std_msgs::UInt8MultiArray msg_in;

// Protocol_bridge -> Hardware bridge
std_msgs::UInt8MultiArray msg_out;

const uint64_t receiveDeadtime = 100;
bool isTopicUpdated = false;

/**
 * Closes port if it is closed, initialized it
 * with given parameter and DOES NOT OPEN IT.
 */
void initPort(serial::Serial &port, std::string device,
              int baudrate, int timeout, serial::bytesize_t dataBytes, serial::parity_t parity,
              serial::stopbits_t stopBits) {
  if (port.isOpen())
    port.close();

  port.setPort(device);
  serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(timeout);
  port.setTimeout(serialTimeout);
  port.setBaudrate(baudrate);
  port.setBytesize(dataBytes);
  port.setParity(parity);
  port.setStopbits(stopBits);
}

bool sendData(serial::Serial &port) {
  std::vector<uint8_t> msg;

  for (int i = 0; i < RequestMessage::length; i++) {
    msg.push_back(msg_in.data[i]);
  }
  size_t toWrite = sizeof(uint8_t) * msg.size(); // TODO: Check this!

  port.flush();
  size_t written = port.write(msg);

  return written == toWrite;
}

bool receiveData(serial::Serial &port) {
  if (port.available() < ResponseMessage::length) {
    return false;
  }

  std::vector<uint8_t> answer;
  port.read(answer, ResponseMessage::length);

  msg_out.data.clear();
  for (int i = 0; i < ResponseMessage::length; i++) {
    msg_out.data.push_back(answer[i]);
  }

  return true;
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
void inputMessageCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg) {
  msg_in.data.clear();
  for (int i = 0; i < RequestMessage::length; i++) {
    msg_in.data.push_back(msg->data[i]);
  }
  isTopicUpdated = true;
}

int main(int argc, char **argv) {
  /* Initializing node and parameters */
  ros::init(argc, argv, UART_BRIDGE_NODE_NAME);
  ros::NodeHandle nodeHandle(UART_BRIDGE_NODE_NAME);

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
      ROS_ERROR("Forbidden data bytes size %d, available sizes: 5, 6, 7, 8", dataBytesInt);
      return 0;
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
    ROS_ERROR("Urecognised parity \"%s\", available parities: \"none\", \"odd\", \"even\"", parityStr.c_str());
    return 0;
  }

  int stopBitsInt;
  nodeHandle.param(PARAM_STOP_BITS, stopBitsInt, DEFAULT_STOP_BITS);
  serial::stopbits_t stopBits;
  switch (stopBitsInt) {
    case 1:stopBits = serial::stopbits_t::stopbits_one;
      break;
    case 2:stopBits = serial::stopbits_t::stopbits_two;
      break;
    default:ROS_ERROR("Forbidden stop bits size %d, available sizes: 1, 2", stopBitsInt);
      return 0;
  }

  msg_in.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_in.layout.dim[0].size = RequestMessage::length;
  msg_in.layout.dim[0].stride = RequestMessage::length;
  msg_in.layout.dim[0].label = "msg_in";

  msg_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_out.layout.dim[0].size = ResponseMessage::length;
  msg_out.layout.dim[0].stride = ResponseMessage::length;
  msg_out.layout.dim[0].label = "msg_out";

  ros::Publisher outputMessage_pub = nodeHandle.advertise<std_msgs::UInt8MultiArray>(INPUT_PARCEL_TOPIC, 100);

  ros::Subscriber inputMessage_sub = nodeHandle.subscribe(OUTPUT_PARCEL_TOPIC, 100, inputMessageCallback);

  ros::Rate readDelay(100);

  // Initialasing serial port
  serial::Serial port;
  ROS_INFO("UART settings: Device: %s, Baudrate: %d, Data bytes: %d, Parity: %s, Stop bits: %d",
           device.c_str(), baudrate, dataBytes, parityStr.c_str(), stopBitsInt);
  initPort(port, device, baudrate, DEFAULT_SERIAL_TIMEOUT, dataBytes, parity, stopBits);

  while (ros::ok()) {
    // TODO: Add exception handling and ROS diagnostics support

    if (!port.isOpen()) {
      port.open();
      if (!port.isOpen()) {
        ROS_ERROR("Unable to open UART port");
        ros::spinOnce();
        continue;
      }
    }

    // If topic is sent us something
    if (isTopicUpdated) {

      if (!sendData(port)) {
        ROS_ERROR("Unable to send msg to STM32");
      }

      readDelay.sleep();

      if (receiveData(port)) {
        outputMessage_pub.publish(msg_out);
      } else {
        ROS_ERROR("Unable to receive msg from STM32");
      }

      isTopicUpdated = false;
    }

    ros::spinOnce();
  }

  return 0;
}
