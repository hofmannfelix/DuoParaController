#include "MadCyphal.h"

// Encode
unsigned long encodeCyphalFrameId(uint8_t priority, uint8_t nodeId, uint16_t subjectId)
{
    unsigned long result = 0;

    // Ensure priority is within 3 bits (0-7), nodeId is within 3 bits (0-7), and subjectId is within 13 bits (0-8191)
    priority = priority & 0x07;      // Priority fits into 3 bits
    nodeId = nodeId & 0x07;          // Node ID fits into 3 bits (bits 5-7)
    subjectId = subjectId & 0x1FFF;  // Subject ID fits into 13 bits (bits 8-20)

    // Combine nodeId and subjectId into a 32-bit integer
    result = (subjectId << 8) | nodeId;

    // Clear bit 25 as message frame
    result &= ~(1 << 25);

    // Set bit 7 to 0
    result &= ~(1 << 7);

    // Set bit 23 to 0
    result &= ~(1 << 23);

    // Set bits 21 and 22 to 1
    result |= (1 << 21) | (1 << 22);

    // Set priority in bits 26 to 28
    result |= (priority << 26);

    return result;
}

unsigned long encodeCyphalServiceFrameId(uint8_t priority, uint8_t nodeId, uint16_t destinationId, uint16_t serviceId)
{
    unsigned long result = 0;

    // Ensure priority is within 3 bits (0-7), nodeId is within 3 bits (0-7), and subjectId is within 13 bits (0-8191)
    priority &= 0x07;       // Priority fits into 3 bits
    
    // Ensure the inputs are within valid bit ranges
    nodeId &= 0x7F;         // 7 bits for nodeId (0 to 6)
    destinationId &= 0x7F;  // 7 bits for destinationId (7 to 13)
    serviceId &= 0x1FF;     // 9 bits for serviceId (14 to 22)

    // Combine the values into a single result
    result = (serviceId << 14) | (destinationId << 7) | nodeId;

    // set bit 24 as service request
    result |= (1 << 24);

    // set bit 25 as service frame
    result |= (1 << 25);

    // Set priority in bits 26 to 28
    result |= (priority << 26);

    return result;
}

uint8_t trailingByteSingleFrame() {
    static uint8_t counter = 0;

    // Mask for setting bits 5 to 7 to 1 (0b11100000 or 0xE0)
    uint8_t result = (counter & 0x1F) | 0xE0; // Counter in bits 0-4, bits 5-7 set to 1

    // Increment the counter and wrap around at 31
    counter = (counter + 1) % 32;

    return result;
}

void encodeNewNodeId(unsigned char *buffer, int nodeId)
{
  buffer[0] = 0;
  buffer[1] = static_cast<uint8_t>(nodeId);
  buffer[2] = trailingByteSingleFrame();
}

void encodeThrottleValues(uint16_t *throt, uint8_t *throtOut) {
  /* Remove the upper two digits */
  throt[0] &= 0x3fffu;
  throt[1] &= 0x3fffu;
  throt[2] &= 0x3fffu;
  throt[3] &= 0x3fffu;
  /* Split the upper 6 bits of the last throttle */
  throt[0] |= ((throt[3]<<2)&0xc000u);
  throt[1] |= ((throt[3]<<4)&0xc000u);
  throt[2] |= ((throt[3]<<6)&0xc000u);
  /* Copy data */
  *(uint16_t *)(&throtOut[0]) = throt[0];
  *(uint16_t *)(&throtOut[2]) = throt[1];
  *(uint16_t *)(&throtOut[4]) = throt[2];
  *(uint16_t *)(&throtOut[6]) = throt[3];

  throtOut[7] = trailingByteSingleFrame();
}

// Decode
void decodeCyphalFrameId(uint32_t input, uint8_t &nodeId, uint16_t &subjectId)
{
    // Mask for Node ID: Bits 0-7 (0xFF is 8 bits all set to 1, or 11111111 in binary)
    nodeId = input & 0xFF;

    // Mask for Subject ID: Bits 8-20 (13 bits) -> shift input right by 8 bits, then mask with 0x1FFF (13 bits)
    subjectId = (input >> 8) & 0x1FFF;
}

void decodeBufferCurrentInfo(const unsigned char *buffer, int &electricalSpeed, int &busCurrent, uint16_t &operatingStatus)
{
    // Decode electricalSpeed (2 bytes: buffer[0] and buffer[1])
    electricalSpeed = (static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8)) / 10;

    // Decode busCurrent (2 bytes: buffer[2] and buffer[3])
    busCurrent = (static_cast<uint16_t>(buffer[2]) | (static_cast<uint16_t>(buffer[3]) << 8)) / 10;

    // Decode operatingStatus (2 bytes: buffer[4] and buffer[5])
    operatingStatus = static_cast<uint16_t>(buffer[4]) | (static_cast<uint16_t>(buffer[5]) << 8);
}

void decodeBufferVoltageInfo(const unsigned char *buffer, int &outputThrottle, float &busVoltage, int &mosTemp, int &capacitanceTemp, int &motorTemp)
{
    // Decode outputThrottle (2 bytes: buffer[0] and buffer[1])
    outputThrottle = static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);

    // Decode busVoltage (2 bytes: buffer[2] and buffer[3])
    busVoltage = (static_cast<uint16_t>(buffer[2]) | (static_cast<uint16_t>(buffer[3]) << 8)) / 10;

    // Decode mosTemp (1 byte: buffer[4])
    mosTemp = buffer[4] - 40;

    // Decode capacitanceTemp (1 byte: buffer[5])
    capacitanceTemp = buffer[5] - 40;

    // Decode motorTemp (1 byte: buffer[6])
    motorTemp = buffer[6] - 40;
}