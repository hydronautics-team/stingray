#ifndef STINGRAY_MESSAGES_H
#define STINGRAY_MESSAGES_H

#include <stdint.h>
#include <unistd.h>
#include <vector>

/** Bits to initialize stabilization
 */
#define SHORE_STABILIZE_DEPTH_BIT       0
#define SHORE_STABILIZE_ROLL_BIT        1
#define SHORE_STABILIZE_PITCH_BIT       2
#define SHORE_STABILIZE_YAW_BIT         3
#define SHORE_STABILIZE_LAG_BIT         4
#define SHORE_STABILIZE_MARCH_BIT       5
#define SHORE_STABILIZE_IMU_BIT         6
#define SHORE_STABILIZE_SAVE_BIT        7

/// Number of the propellers
static const uint8_t VmaAmount = 8;
/// Number of the control constants
static const uint8_t ControlAmount = 7;
/// Number of the devs
static const uint8_t DevAmount = 6;

/** Enumerator for constants in the automatic control system
 */
enum ControlConstantNames {
    CONTROL_K1 = 0,
    CONTROL_K2,
    CONTROL_K3,
    CONTROL_K4,
    CONTROL_IBORDERS,
    CONTROL_PGAIN,
    CONTROL_IGAIN
};

/** Enumerator for devs names
 */
enum DevNames {
    DEV_LIGHT = 0,
    DEV_TILT,
    DEV_GRAB,
    DEV_GRAB_ROTATE,
    DEV_ADDITIONAL_1,
    DEV_ADDITIONAL_2
};

/** @brief Structure for storing and processing data from the STM32 normal request message protocol
 * Shore send requests and STM send responses
 */
struct RequestMessage
{
    RequestMessage();

    /// Length in bytes of the normal message protocol
    const static uint8_t length = 30;

    /// Type code for the normal message protocol
    const static uint8_t type = 0xA5;
    uint8_t flags;
    int16_t march;
    int16_t lag;
    int16_t depth;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int8_t dev[DevAmount];
    int32_t lag_error;
    uint8_t dev_flags;
    uint8_t stabilize_flags;
    uint8_t cameras;
    uint8_t pc_reset;
    //uint16_t checksum;

    std::vector<uint8_t> formVector();
};

/** @brief Structure for storing and processing data from the STM32 configuration request message protocol
 * Shore send requests and STM send responses
 */
struct ConfigRequestMessage
{
    ConfigRequestMessage();

    /// Length in bytes of the configuration message protocol
    const static uint8_t length = 195;

    /// Type code for the configuration message protocol
    const static uint8_t type = 0x55;
    float depth_control[ControlAmount];
    float roll_control[ControlAmount];
    float pitch_control[ControlAmount];
    float yaw_control[ControlAmount];

    uint8_t vma_position[VmaAmount];
    uint8_t vma_setting[VmaAmount];
    uint8_t vma_kforward[VmaAmount];
    uint8_t vma_kbackward[VmaAmount];

    //uint16_t checksum;

    std::vector<uint8_t> formVector();
};

/** @brief Structure for storing and processing data from the STM32 configuration response message protocol
 * Shore send requests and STM send responses
 */
struct ResponseMessage
{
    ResponseMessage();

    const static uint8_t length = 70;

    float roll;
    float pitch;
    float yaw;

    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;

    float depth;
    //float lag;
    float in_pressure;

    uint8_t dev_state;
    int16_t leak_data;

    uint16_t vma_current[VmaAmount];
    uint16_t dev_current[DevAmount];

    uint16_t vma_errors;
    uint16_t dev_errors;
    uint8_t pc_errors;

    uint16_t checksum;

    bool parseVector(std::vector<uint8_t> &input);
};

void pushToVector(std::vector<uint8_t> &vector, int8_t var);
void pushToVector(std::vector<uint8_t> &vector, uint8_t var);
void pushToVector(std::vector<uint8_t> &vector, int16_t var, bool revert = false);
void pushToVector(std::vector<uint8_t> &vector, uint16_t var, bool revert = false);
void pushToVector(std::vector<uint8_t> &vector, float var, bool revert = false);
void pushToVector(std::vector<uint8_t> &vector, int32_t var, bool revert = false);

void popFromVector(std::vector<uint8_t> &container, int8_t &output);
void popFromVector(std::vector<uint8_t> &container, uint8_t &output);
void popFromVector(std::vector<uint8_t> &container, int16_t &output, bool revert = true);
void popFromVector(std::vector<uint8_t> &container, uint16_t &output, bool revert = true);
void popFromVector(std::vector<uint8_t> &container, float &output, bool revert = true);

uint16_t getChecksum16b(std::vector<uint8_t> &msg);

bool pickBit(uint8_t &input, uint8_t bit);
void setBit(uint8_t &byte, uint8_t bit, bool state);
void setStabilizationState(RequestMessage& request, uint8_t bit, bool state);

#endif //STINGRAY_MESSAGES_H
