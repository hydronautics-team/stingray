#ifndef STINGRAY_MESSAGES_COMMON_H
#define STINGRAY_MESSAGES_COMMON_H

#include <stdint.h>
#include <unistd.h>

#include <fstream>
#include <vector>

#include "stingray_utils/json.hpp"

#define UNUSED(x) (void)(x)
using json = nlohmann::json;

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
