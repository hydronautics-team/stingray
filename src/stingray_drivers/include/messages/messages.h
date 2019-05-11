#ifndef STINGRAY_MESSAGES_H
#define STINGRAY_MESSAGES_H

#include <stdint.h>
#include <unistd.h>
#include <vector>


/** Enumerator for VMA names
 * H - horizontal, V - verical
 * L - left, R - right
 * B - back, F - forward
 */
enum VmaNames {
    VMA_HLB = 0,
    VMA_HLF,
    VMA_HRB,
    VMA_HRF,
    VMA_VB,
    VMA_VF,
    VMA_VL,
    VMA_VR
};
/// Number of the propellers
static const uint8_t VmaAmount = 8;

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
/// Number of the control constants
static const uint8_t ControlAmount = 7;

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
/// Number of the devs
static const uint8_t DevAmount = 6;

void pushToVector(std::vector<uint8_t> &vector, int8_t var);
void pushToVector(std::vector<uint8_t> &vector, uint8_t var);
void pushToVector(std::vector<uint8_t> &vector, int16_t var, bool revert = false);
void pushToVector(std::vector<uint8_t> &vector, uint16_t var, bool revert = false);
void pushToVector(std::vector<uint8_t> &vector, float var, bool revert = false);

void popFromVector(std::vector<uint8_t> &container, int8_t &output);
void popFromVector(std::vector<uint8_t> &container, uint8_t &output);
void popFromVector(std::vector<uint8_t> &container, int16_t &output, bool revert = true);
void popFromVector(std::vector<uint8_t> &container, uint16_t &output, bool revert = true);
void popFromVector(std::vector<uint8_t> &container, float &output, bool revert = true);

uint16_t getChecksum16b(std::vector<uint8_t> &msg);

/** @brief Structure for storing and processing data from the STM32 normal request message protocol
 * Shore send requests and STM send responses
 */
struct RequestMessage
{
    RequestMessage();

    /// Length in bytes of the normal message protocol
    const static uint8_t length = 26;

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

/* Direct mode */
#define REQUEST_DIRECT_CODE             0xAA
#define REQUEST_DIRECT_LENGTH           11

#define REQUEST_DIRECT_TYPE             0
#define REQUEST_DIRECT_1                1
#define REQUEST_DIRECT_2                2
#define REQUEST_DIRECT_3                3
#define REQUEST_DIRECT_4                4
#define REQUEST_DIRECT_5                5
#define REQUEST_DIRECT_6                6
#define REQUEST_DIRECT_7                7
#define REQUEST_DIRECT_8                8
#define REQUEST_DIRECT_CHECKSUM         9

#endif //STINGRAY_MESSAGES_H
