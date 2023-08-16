#include "messages/config.h"

RequestConfigMessage::RequestConfigMessage() : AbstractMessage() {
    contour = 0;
    march = 0;
    lag = 0;
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    pJoyUnitCast = 0;
    pSpeedDyn = 0;
    pErrGain = 0;
    posFilterT = 0;
    posFilterK = 0;
    speedFilterT = 0;
    speedFilterK = 0;
    pid_pGain = 0;
    pid_iGain = 0;
    pid_iMax = 0;
    pid_iMin = 0;
    pThrustersMin = 0;
    pThrustersMax = 0;
    thrustersFilterT = 0;
    thrustersFilterK = 0;
    sOutSummatorMax = 0;
    sOutSummatorMin = 0;

    checksum = 0;
}

void RequestConfigMessage::serialize(std::vector<uint8_t>& container) {
    pushToVector(container, type);
    pushToVector(container, contour);
    pushToVector(container, march);
    pushToVector(container, lag);
    pushToVector(container, depth);
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);
    pushToVector(container, pJoyUnitCast);
    pushToVector(container, pSpeedDyn);
    pushToVector(container, pErrGain);
    pushToVector(container, posFilterT);
    pushToVector(container, posFilterK);
    pushToVector(container, speedFilterT);
    pushToVector(container, speedFilterK);
    pushToVector(container, pid_pGain);
    pushToVector(container, pid_iGain);
    pushToVector(container, pid_iMax);
    pushToVector(container, pid_iMin);
    pushToVector(container, pThrustersMin);
    pushToVector(container, pThrustersMax);
    pushToVector(container, thrustersFilterT);
    pushToVector(container, thrustersFilterK);
    pushToVector(container, sOutSummatorMax);
    pushToVector(container, sOutSummatorMin);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);  // do i need to revert bytes here?
}


/** @brief Constructor for ResponseConfigMessage
 *
 */
ResponseConfigMessage::ResponseConfigMessage() : AbstractMessage() {
    code = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    raw_yaw = 0;
    rollSpeed = 0;
    pitchSpeed = 0;
    yawSpeed = 0;
    pressure = 0;
    in_pressure = 0;
    inputSignal = 0;
    speedSignal = 0;
    posSignal = 0;
    joyUnitCasted = 0;
    joy_iValue = 0;
    posError = 0;
    speedError = 0;
    dynSummator = 0;
    pidValue = 0;
    posErrorAmp = 0;
    speedFiltered = 0;
    posFiltered = 0;
    pid_iValue = 0;
    thrustersFiltered = 0;
    outputSignal = 0;

    checksum = 0;
}


bool ResponseConfigMessage::deserialize(std::vector<uint8_t> &input) {
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum) {
        return false;
    }

    popFromVector(input, outputSignal);
    popFromVector(input, thrustersFiltered);
    popFromVector(input, pid_iValue);
    popFromVector(input, posFiltered);
    popFromVector(input, speedFiltered);
    popFromVector(input, posErrorAmp);
    popFromVector(input, pidValue);
    popFromVector(input, dynSummator);
    popFromVector(input, speedError);
    popFromVector(input, posError);
    popFromVector(input, joy_iValue);
    popFromVector(input, joyUnitCasted);
    popFromVector(input, posSignal);
    popFromVector(input, speedSignal);
    popFromVector(input, inputSignal);
    popFromVector(input, in_pressure);
    popFromVector(input, pressure);
    popFromVector(input, yawSpeed);
    popFromVector(input, pitchSpeed);
    popFromVector(input, rollSpeed);
    popFromVector(input, raw_yaw);
    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);
    popFromVector(input, code);

    return true;
}