#include "messages/messages.h"

/** @brief Constructor for RequestMessage
  *
  */
RequestMessage::RequestMessage()
{
    flags = 0;
    march = 0;
    lag = 0;
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    for(int i=0; i<DevAmount; i++) {
        dev[i] = 0;
    }

    dev_flags = 0;
    stabilize_flags = 0;
    cameras = 0;
    pc_reset = 0;
}

/** @brief Form bitwise correct string with computed 16bit checksum from the data stored in RequestMessage
  *lag_error
  */
std::vector<uint8_t> RequestMessage::formVector()
{
    std::vector<uint8_t> container;

    pushToVector(container, type);
    pushToVector(container, flags);

    pushToVector(container, march);
    pushToVector(container, lag);
    pushToVector(container, depth);
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);

    for(int i=0; i<DevAmount; i++) {
        pushToVector(container, dev[i]);
    }

    pushToVector(container, lag_error); 
    pushToVector(container, dev_flags);
    pushToVector(container, stabilize_flags);
    pushToVector(container, cameras);
    pushToVector(container, pc_reset);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum); // do i need to revert bytes here?

    return container;
}

/** @brief Constructor for ConfigRequestMessage
  *
  */
ConfigRequestMessage::ConfigRequestMessage()
{
    for(int i=0; i<ControlAmount; i++) {
        depth_control[i] = 0;
    }

    for(int i=0; i<ControlAmount; i++) {
        roll_control[i] = 0;
    }

    for(int i=0; i<ControlAmount; i++) {
        pitch_control[i] = 0;
    }

    for(int i=0; i<ControlAmount; i++) {
        yaw_control[i] = 0;
    }

    for(int i=0; i<VmaAmount; i++) {
        vma_position[i] = 0;
    }

    for(int i=0; i<VmaAmount; i++) {
        vma_setting[i] = 0;
    }

    for(int i=0; i<VmaAmount; i++) {
        vma_kforward[i] = 0;
    }

    for(int i=0; i<VmaAmount; i++) {
        vma_kbackward[i] = 0;
    }
}

/** @brief Form bitwise correct string with computed 16bit checksum from the data stored in ConfigRequestMessage
  *
  */
std::vector<uint8_t> ConfigRequestMessage::formVector()
{
    std::vector<uint8_t> container;
    pushToVector(container, type);

    for(int i=0; i<ControlAmount; i++) {
        pushToVector(container, depth_control[i]);
    }

    for(int i=0; i<ControlAmount; i++) {
        pushToVector(container, roll_control[i]);
    }

    for(int i=0; i<ControlAmount; i++) {
        pushToVector(container, pitch_control[i]);
    }

    for(int i=0; i<ControlAmount; i++) {
        pushToVector(container, yaw_control[i]);
    }

    for(int i=0; i<VmaAmount; i++) {
        pushToVector(container, vma_position[i]);
    }

    for(int i=0; i<VmaAmount; i++) {
        pushToVector(container, vma_setting[i]);
    }

    for(int i=0; i<VmaAmount; i++) {
        pushToVector(container, vma_kforward[i]);
    }

    for(int i=0; i<VmaAmount; i++) {
        pushToVector(container, vma_kbackward[i]);
    }

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum, false); // do i need to revert bytes here?

    return container;
}

/** @brief Constructor for ResponseMessage
  *
  */
ResponseMessage::ResponseMessage()
{
    roll = 0;
    pitch = 0;
    yaw = 0;

    rollSpeed = 0;
    pitchSpeed = 0;
    yawSpeed = 0;

    depth = 0;
    //lag = 0;
    in_pressure = 0;

    dev_state = 0;
    leak_data = 0;

    for(int i=0; i<VmaAmount; i++) {
        vma_current[i] = 0;
    }

    for(int i=0; i<DevAmount; i++) {
        dev_current[i] = 0;
    }

    vma_errors = 0;
    dev_errors = 0;
    pc_errors = 0;

    checksum = 0;
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
bool ResponseMessage::parseVector(std::vector<uint8_t> &input)
{
    popFromVector(input, checksum, true);

    uint16_t checksum_calc = getChecksum16b(input);

    if(checksum_calc != checksum) {
        return false;
    }

    popFromVector(input, pc_errors);
    popFromVector(input, dev_errors);
    popFromVector(input, vma_errors);

    for(int i=0; i<DevAmount; i++) {
        popFromVector(input, dev_current[DevAmount-i]);
    }

    for(int i=0; i<VmaAmount; i++) {
        popFromVector(input, vma_current[VmaAmount-i]);
    }

    popFromVector(input, leak_data);
    popFromVector(input, dev_state);

    popFromVector(input, in_pressure);
    popFromVector(input, depth);
    //popFromVector(input, lag);

    popFromVector(input, yawSpeed);
    popFromVector(input, pitchSpeed);
    popFromVector(input, rollSpeed);

    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);

    return true;
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
  *
  * @param[in]  var     Variable to transform.
  * @param[in]  revert  Revert bytes or not.
  */
void pushToVector(std::vector<uint8_t> &vector, int8_t var)
{
    uint8_t buf = *reinterpret_cast<uint8_t*>(&var);
    vector.push_back(buf);
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
  *
  * @param[in]  var     Variable to transform.
  * @param[in]  revert  Revert bytes or not.
  */
void pushToVector(std::vector<uint8_t> &vector, uint8_t var)
{
    vector.push_back(var);
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
  *
  * @param[in]  var     Variable to transform.
  * @param[in]  revert  Revert bytes or not.
  */
void pushToVector(std::vector<uint8_t> &vector, int16_t var, bool revert)
{
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&var);
    if(revert) {
        vector.push_back(ptr[1]);
        vector.push_back(ptr[0]);
    }
    else {
        vector.push_back(ptr[0]);
        vector.push_back(ptr[1]);
    }
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
  *
  * @param[in]  var     Variable to transform.
  * @param[in]  revert  Revert bytes or not.
  */
void pushToVector(std::vector<uint8_t> &vector, uint16_t var, bool revert)
{
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&var);
    if(revert) {
        vector.push_back(ptr[1]);
        vector.push_back(ptr[0]);
    }
    else {
        vector.push_back(ptr[0]);
        vector.push_back(ptr[1]);
    }
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
  *
  * @param[in]  var     Variable to transform.
  * @param[in]  revert  Revert bytes or not.
  */
void pushToVector(std::vector<uint8_t> &vector, float var, bool revert)
{
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&var);
    for(int i=0; i<4; i++) {
        if(revert) {
            vector.push_back(ptr[3-i]);
        }
        else {
            vector.push_back(ptr[i]);
        }
    }
}

/** @brief Overloaded transform to string function, transforms value to string bitwise correctly
  *
  * @param[in]  var     Variable to transform.
  * @param[in]  revert  Revert bytes or not.
  */
void pushToVector(std::vector<uint8_t> &vector, int32_t var, bool revert)
{
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&var);
    for(int i=0; i<4; i++) {
        if(revert) {
            vector.push_back(ptr[3-i]);
        }
        else {
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
void popFromVector(std::vector<uint8_t> &vector, uint8_t &output)
{
    output = vector.back();
    vector.pop_back();
}

/** @brief Overloaded pick from string, picks value from the end of the string bitwise correctly
  *
  * @param[out] &container  Link to container string with bytes.
  * @param[out] &value      Link to variable in which the data will be stored.
  * @param[in]  revert      Revert bytes or not.
  */
void popFromVector(std::vector<uint8_t> &vector, int8_t &output)
{
    uint8_t out_raw = vector.back();
    vector.pop_back();
    output = *reinterpret_cast<int8_t*>(&out_raw);
}

/** @brief Overloaded pick from string, picks value from the end of the string bitwise correctly
  *
  * @param[out] &container  Link to container string with bytes.
  * @param[out] &value      Link to variable in which the data will be stored.
  * @param[in]  revert      Revert bytes or not.
  */
void popFromVector(std::vector<uint8_t> &vector, int16_t &output, bool revert)
{
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&output);
    if(revert) {
        ptr[1] = vector.back();
        vector.pop_back();
        ptr[0] = vector.back();
    }
    else {
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
void popFromVector(std::vector<uint8_t> &vector, uint16_t &output, bool revert)
{
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&output);
    if(revert) {
        ptr[1] = vector.back();
        vector.pop_back();
        ptr[0] = vector.back();
    }
    else {
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
void popFromVector(std::vector<uint8_t> &vector, float &output, bool revert)
{
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&output);
    for(size_t i=0; i<4; i++) {
        if(revert) {
            ptr[3-i] = vector.back();
            vector.pop_back();
        }
        else {
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
uint16_t getChecksum16b(std::vector<uint8_t> &vector)
{
    uint16_t len = vector.size();
    uint16_t crc = 0xFFFF;
    uint8_t i;
    uint8_t g = 0;

    while (len--) {
        crc ^= vector[g++] << 8;

        for (i = 0; i < 8; i++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

bool pickBit(uint8_t &input, uint8_t bit)
{
    return static_cast<bool>((input << (8 - bit)) >> 8);
}

void setBit(uint8_t &byte, uint8_t bit, bool state)
{
    uint8_t value = 1;
    if(state) {
        byte = byte | (value << bit);
    }
    else {
        byte = byte & ~(value << bit);
    }
}

void setStabilizationState(RequestMessage& request, uint8_t bit, bool state)
{
    setBit(request.stabilize_flags, bit, state);
}
