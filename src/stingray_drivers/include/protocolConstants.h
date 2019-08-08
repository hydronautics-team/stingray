#ifndef STINGRAY_DRIVERS_PROTOCOLCONSTANTS_H
#define STINGRAY_DRIVERS_PROTOCOLCONSTANTS_H

#include <string>

// TODO: Documentation
static const std::string    UART_DRIVER_NODE_NAME   = "uart_driver";
static const std::string    PARAM_DEVICE            = "device";
static const std::string    PARAM_BAUDRATE          = "baudrate";
static const std::string    PARAM_DATA_BYTES        = "dataBytes";
static const std::string    PARAM_PARITY            = "parity";
static const std::string    PARAM_STOP_BITS         = "stopBits";
static const std::string    PARITY_NONE             = "none";
static const std::string    PARITY_EVEN             = "even";
static const std::string    PARITY_ODD              = "odd";
static const std::string    DEFAULT_DEVICE          = "/dev/ttyS0";
static const int            DEFAULT_BAUDRATE        = 57600;
static const int            DEFAULT_DATA_BYTES      = 8;
static const std::string    DEFAULT_PARITY          = PARITY_NONE;
static const int            DEFAULT_STOP_BITS       = 1;
static const int            DEFAULT_SERIAL_TIMEOUT  = 1000; // Needed for serial port library

#endif //STINGRAY_DRIVERS_PROTOCOLCONSTANTS_H
