#include "messages/common.h"

AbstractMessage::AbstractMessage() {
    // parse json config
    config = json::parse(std::ifstream("resources/configs/communication.json"));
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
 *
 * @param[in]  var     Variable to transform.
 * @param[in]  revert  Revert bytes or not.
 */
void pushToVector(std::vector<uint8_t> &vector, int8_t var) {
    uint8_t buf = *reinterpret_cast<uint8_t *>(&var);
    vector.push_back(buf);
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
 *
 * @param[in]  var     Variable to transform.
 * @param[in]  revert  Revert bytes or not.
 */
void pushToVector(std::vector<uint8_t> &vector, uint8_t var) { vector.push_back(var); }

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
 *
 * @param[in]  var     Variable to transform.
 * @param[in]  revert  Revert bytes or not.
 */
void pushToVector(std::vector<uint8_t> &vector, int16_t var, bool revert) {
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&var);
    if (revert) {
        vector.push_back(ptr[1]);
        vector.push_back(ptr[0]);
    } else {
        vector.push_back(ptr[0]);
        vector.push_back(ptr[1]);
    }
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
 *
 * @param[in]  var     Variable to transform.
 * @param[in]  revert  Revert bytes or not.
 */
void pushToVector(std::vector<uint8_t> &vector, uint16_t var, bool revert) {
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&var);
    if (revert) {
        vector.push_back(ptr[1]);
        vector.push_back(ptr[0]);
    } else {
        vector.push_back(ptr[0]);
        vector.push_back(ptr[1]);
    }
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
 *
 * @param[in]  var     Variable to transform.
 * @param[in]  revert  Revert bytes or not.
 */
void pushToVector(std::vector<uint8_t> &vector, float var, bool revert) {
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&var);
    for (int i = 0; i < 4; i++) {
        if (revert) {
            vector.push_back(ptr[3 - i]);
        } else {
            vector.push_back(ptr[i]);
        }
    }
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
 *
 * @param[in]  var     Variable to transform.
 * @param[in]  revert  Revert bytes or not.
 */
void pushToVector(std::vector<uint8_t> &vector, int32_t var, bool revert) {
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&var);
    for (int i = 0; i < 4; i++) {
        if (revert) {
            vector.push_back(ptr[3 - i]);
        } else {
            vector.push_back(ptr[i]);
        }
    }
}

/** @brief Overloaded pick from string, picks value from the end of the string bitwise correctly
 *
 * @param[out] &container  Link to container string with bytes.
 * @param[out] &value      Link to variable in which the data will be stored.
 * @param[in]  revert      Revert bytes or not.
 */
void popFromVector(std::vector<uint8_t> &vector, uint8_t &output) {
    output = vector.back();
    vector.pop_back();
}

/** @brief Overloaded pick from string, picks value from the end of the string bitwise correctly
 *
 * @param[out] &container  Link to container string with bytes.
 * @param[out] &value      Link to variable in which the data will be stored.
 * @param[in]  revert      Revert bytes or not.
 */
void popFromVector(std::vector<uint8_t> &vector, int8_t &output) {
    uint8_t out_raw = vector.back();
    vector.pop_back();
    output = *reinterpret_cast<int8_t *>(&out_raw);
}

/** @brief Overloaded pick from string, picks value from the end of the string bitwise correctly
 *
 * @param[out] &container  Link to container string with bytes.
 * @param[out] &value      Link to variable in which the data will be stored.
 * @param[in]  revert      Revert bytes or not.
 */
void popFromVector(std::vector<uint8_t> &vector, int16_t &output, bool revert) {
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&output);
    if (revert) {
        ptr[1] = vector.back();
        vector.pop_back();
        ptr[0] = vector.back();
    } else {
        ptr[0] = vector.back();
        vector.pop_back();
        ptr[1] = vector.back();
    }
    vector.pop_back();
}

/** @brief Overloaded pick from string, picks value from the end of the string bitwise correctly
 *
 * @param[out] &container  Link to container string with bytes.
 * @param[out] &value      Link to variable in which the data will be stored.
 * @param[in]  revert      Revert bytes or not.
 */
void popFromVector(std::vector<uint8_t> &vector, uint16_t &output, bool revert) {
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&output);
    if (revert) {
        ptr[1] = vector.back();
        vector.pop_back();
        ptr[0] = vector.back();
    } else {
        ptr[0] = vector.back();
        vector.pop_back();
        ptr[1] = vector.back();
    }
    vector.pop_back();
}

/** @brief Overloaded pick from string, picks value from the end of the string bitwise correctly
 *
 * @param[out] &container  Link to container string with bytes.
 * @param[out] &value      Link to variable in which the data will be stored.
 * @param[in]  revert      Revert bytes or not.
 */
void popFromVector(std::vector<uint8_t> &vector, float &output, bool revert) {
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&output);
    for (size_t i = 0; i < 4; i++) {
        if (revert) {
            ptr[3 - i] = vector.back();
            vector.pop_back();
        } else {
            ptr[i] = vector.back();
            vector.pop_back();
        }
    }
}

/** @brief Gets 16 bit checksum for the content of the stream
 *
 * @param[in]  &msg    Link to the stream
 * @param[in]  length  Length of the message in the stream
 */
uint16_t getChecksum16b(std::vector<uint8_t> &vector) {
    uint16_t len = vector.size();
    uint16_t crc = 0xFFFF;
    uint8_t i;
    uint8_t g = 0;

    while (len--) {
        crc ^= vector[g++] << 8;

        for (i = 0; i < 8; i++) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

bool pickBit(uint8_t &input, uint8_t bit) { return static_cast<bool>((input << (8 - bit)) >> 8); }

void setBit(uint8_t &byte, uint8_t bit, bool state) {
    uint8_t value = 1;
    if (state) {
        byte = byte | (value << bit);
    } else {
        byte = byte & ~(value << bit);
    }
}