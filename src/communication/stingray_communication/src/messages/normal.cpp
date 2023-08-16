#include "messages/normal.h"

/** @brief Constructor for RequestNormalMessage
 *
 */
RequestNormalMessage::RequestNormalMessage() : AbstractMessage() {
    flags = 0;
    march = 0;
    lag = 0;
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    for (int i = 0; i < DevAmount; i++) {
        dev[i] = 0;
    }

    dev_flags = 0;
    stabilize_flags = 0;
    cameras = 0;
    pc_reset = 0;
    checksum = 0;
}

/** @brief Form bitwise correct string with computed 16bit checksum from the data stored in RequestNormalMessage
 *lag_error
 */
void RequestNormalMessage::serialize(std::vector<uint8_t> &container) {
    pushToVector(container, type);
    pushToVector(container, flags);

    pushToVector(container, march);
    pushToVector(container, lag);
    pushToVector(container, depth);
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);

    for (int i = 0; i < DevAmount; i++) {
        pushToVector(container, dev[i]);
    }

    pushToVector(container, lag_error);
    pushToVector(container, dev_flags);
    pushToVector(container, stabilize_flags);
    pushToVector(container, cameras);
    pushToVector(container, pc_reset);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);  // do i need to revert bytes here?
}

/**
 * @brief Set stabilization state for a specific bit
 * @param[in] bit Bit to set
 * @param[in] state State to set
 */
void RequestNormalMessage::setStabilizationState(uint8_t bit, bool state) { setBit(stabilize_flags, bit, state); }

/** @brief Constructor for ResponseNormalMessage
 *
 */
ResponseNormalMessage::ResponseNormalMessage() {
    roll = 0;
    pitch = 0;
    yaw = 0;

    rollSpeed = 0;
    pitchSpeed = 0;
    yawSpeed = 0;

    depth = 0;
    // lag = 0;
    in_pressure = 0;

    dev_state = 0;
    leak_data = 0;

    for (int i = 0; i < VmaAmount; i++) {
        vma_current[i] = 0;
    }

    for (int i = 0; i < DevAmount; i++) {
        dev_current[i] = 0;
    }

    vma_errors = 0;
    dev_errors = 0;
    pc_errors = 0;

    checksum = 0;
}

/** @brief Parse string bitwise correctly into ResponseNormalMessage and check 16bit checksum.
 *
 * @param[in]  &input String to parse.
 */
bool ResponseNormalMessage::deserialize(std::vector<uint8_t> &input) {
    popFromVector(input, checksum, true);

    uint16_t checksum_calc = getChecksum16b(input);

    if (checksum_calc != checksum) {
        return false;
    }

    popFromVector(input, pc_errors);
    popFromVector(input, dev_errors);
    popFromVector(input, vma_errors);

    for (int i = 0; i < DevAmount; i++) {
        popFromVector(input, dev_current[DevAmount - i]);
    }

    for (int i = 0; i < VmaAmount; i++) {
        popFromVector(input, vma_current[VmaAmount - i]);
    }

    popFromVector(input, leak_data);
    popFromVector(input, dev_state);

    popFromVector(input, in_pressure);
    popFromVector(input, depth);
    // popFromVector(input, lag);

    popFromVector(input, yawSpeed);
    popFromVector(input, pitchSpeed);
    popFromVector(input, rollSpeed);

    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);

    return true;
}
