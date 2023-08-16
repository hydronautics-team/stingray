#ifndef STINGRAY_MESSAGES_COMMON_H
#define STINGRAY_MESSAGES_COMMON_H

#include <stdint.h>
#include <unistd.h>

#include <fstream>
#include <vector>

#include "stingray_utils/json.hpp"

#define UNUSED(x) (void)(x)
using json = nlohmann::json;

/** Bits to initialize stabilization
 */
#define SHORE_STABILIZE_DEPTH_BIT 0
#define SHORE_STABILIZE_ROLL_BIT 1
#define SHORE_STABILIZE_PITCH_BIT 2
#define SHORE_STABILIZE_YAW_BIT 3
#define SHORE_STABILIZE_LAG_BIT 4
#define SHORE_STABILIZE_MARCH_BIT 5
#define SHORE_STABILIZE_IMU_BIT 6
#define SHORE_STABILIZE_SAVE_BIT 7


/// Number of the propellers
static const uint8_t VmaAmount = 8;
/// Number of the control constants
static const uint8_t ControlAmount = 7;
/// Number of the devs
static const uint8_t DevAmount = 6;

/** Enumerator for constants in the automatic control system
 */
enum ControlConstantNames { CONTROL_K1 = 0, CONTROL_K2, CONTROL_K3, CONTROL_K4, CONTROL_IBORDERS, CONTROL_PGAIN, CONTROL_IGAIN };

/** Enumerator for devs names
 */
enum DevNames { DEV_LIGHT = 0, DEV_TILT, DEV_GRAB, DEV_GRAB_ROTATE, DEV_ADDITIONAL_1, DEV_ADDITIONAL_2 };

/** Abstract class for all messages
 */
struct AbstractMessage {
    AbstractMessage();

    virtual void serialize(std::vector<uint8_t> &container){
        UNUSED(container);
    };
    virtual bool deserialize(std::vector<uint8_t> &container){
        UNUSED(container);
        return false;
    };

    json config;
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

#endif  // STINGRAY_MESSAGES_COMMON_H
