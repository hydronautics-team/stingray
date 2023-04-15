#ifndef STINGRAY_MESSAGES_CONFIG_H
#define STINGRAY_MESSAGES_CONFIG_H

#include "messages/common.h"

struct RequestConfigMessage : public AbstractMessage {
    RequestConfigMessage();

    /// Type code for the Config message protocol
    const static uint8_t type = 0x55;

    uint8_t contour;
    int16_t march;
    int16_t lag;
    int16_t depth;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    float pJoyUnitCast;
    float pSpeedDyn;
    float pErrGain;
    float posFilterT;
    float posFilterK;
    float speedFilterT;
    float speedFilterK;
    float pid_pGain;
    float pid_iGain;
    float pid_iMax;
    float pid_iMin;
    float pThrustersMin;
    float pThrustersMax;
    float thrustersFilterT;
    float thrustersFilterK;
    float sOutSummatorMax;
    float sOutSummatorMin;

    uint16_t checksum;

    void serialize(std::vector<uint8_t> &container) override;
};

/** @brief Structure for storing and processing data from the STM32 configuration request message protocol
 * Shore send requests and STM send responses
 */
struct ResponseConfigMessage : public AbstractMessage {
    ResponseConfigMessage();

    uint8_t code;
    float roll;
    float pitch;
    float yaw;
    float raw_yaw;
    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;
    float pressure;
    float in_pressure;
    float inputSignal;
    float speedSignal;
    float posSignal;
    float joyUnitCasted;
    float joy_iValue;
    float posError;
    float speedError;
    float dynSummator;
    float pidValue;
    float posErrorAmp;
    float speedFiltered;
    float posFiltered;
    float pid_iValue;
    float thrustersFiltered;
    float outputSignal;

    uint16_t checksum;

    bool deserialize(std::vector<uint8_t> &container) override;
};

#endif  // STINGRAY_MESSAGES_CONFIG_H
