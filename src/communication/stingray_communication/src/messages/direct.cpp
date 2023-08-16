#include "messages/direct.h"

RequestDirectMessage::RequestDirectMessage() : AbstractMessage() {
    number = 0;
    id = 0;

    velocity = 0;

    reverse = 0;
    kForward = 0;
    kBackward = 0;

    sForward = 0;
    sBackward = 0;

    checksum = 0;
}

void RequestDirectMessage::serialize(std::vector<uint8_t>& container) {
    pushToVector(container, type);
    pushToVector(container, number);
    pushToVector(container, id);
    pushToVector(container, velocity);
    pushToVector(container, reverse);
    pushToVector(container, kForward);
    pushToVector(container, kBackward);
    pushToVector(container, sForward);
    pushToVector(container, sBackward);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);  // do i need to revert bytes here?
}

/** @brief Constructor for ResponseDirectMessage
 *
 */
ResponseDirectMessage::ResponseDirectMessage() {
    number = 0;
    connection = 0;
    current = 0;

    checksum = 0;
}


bool ResponseDirectMessage::deserialize(std::vector<uint8_t> &input) {
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum) {
        return false;
    }

    popFromVector(input, current);
    popFromVector(input, connection);
    popFromVector(input, number);

    return true;
}