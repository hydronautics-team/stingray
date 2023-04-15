#ifndef STINGRAY_MESSAGES_DIRECT_H
#define STINGRAY_MESSAGES_DIRECT_H

#include "messages/common.h"

struct RequestDirectMessage : public AbstractMessage {
    RequestDirectMessage();
    /// Type code for the Direct message protocol
    const static uint8_t type = 0xAA;
    uint8_t number;
    uint8_t id;
    int8_t velocity;
    uint8_t reverse;
    float kForward;
    float kBackward;
    int8_t sForward;
    int8_t sBackward;

    uint16_t checksum;

    void serialize(std::vector<uint8_t> &container) override;
};

/** @brief Structure for storing and processing data from the STM32 direct request message protocol
 * Shore send requests and STM send responses
 */
struct ResponseDirectMessage : public AbstractMessage {
    ResponseDirectMessage();

    uint8_t number;
    uint8_t connection;
    uint16_t current;

    uint16_t checksum;

    bool deserialize(std::vector<uint8_t> &container) override;
};
#endif  // STINGRAY_MESSAGES_DIRECT_H
