#ifndef STINGRAY_MESSAGES_NORMAL_H
#define STINGRAY_MESSAGES_NORMAL_H

#include "messages/common.h"

/** @brief Structure for storing and processing data from the STM32 normal request message protocol
 * Shore send requests and STM send responses
 */
struct RequestNormalMessage : public AbstractMessage {
    RequestNormalMessage();

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
    uint16_t checksum;

    void serialize(std::vector<uint8_t> &container) override;
    void setStabilizationState(uint8_t bit, bool state);
};

/** @brief Structure for storing and processing data from the STM32 configuration response message protocol
 * Shore send requests and STM send responses
 */
struct ResponseNormalMessage : public AbstractMessage {
    ResponseNormalMessage();

    const static uint8_t length = 70;

    float roll;
    float pitch;
    float yaw;

    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;

    float depth;
    // float lag;
    float in_pressure;

    uint8_t dev_state;
    int16_t leak_data;

    uint16_t vma_current[VmaAmount];
    uint16_t dev_current[DevAmount];

    uint16_t vma_errors;
    uint16_t dev_errors;
    uint8_t pc_errors;

    uint16_t checksum;

    bool deserialize(std::vector<uint8_t> &input) override;
};

#endif  // STINGRAY_MESSAGES_NORMAL_H
