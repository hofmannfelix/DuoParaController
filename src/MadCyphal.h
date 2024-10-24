#ifndef MADCYPHAL_H
#define MADCYPHAL_H

#include <stdint.h>

#define CAN_PRIO_HIGH 1

#define BROADCAST_NODE_ID 126

#define CURRENT_INFO_CAN_ID 6160
#define VOLTAGE_INFO_CAN_ID 6161

#define SET_NODE_ID_CAN_ID 6145
#define SEND_PWM_CAN_ID 6152

// Encoding functions
unsigned long encodeCyphalFrameId(uint8_t priority, uint8_t nodeId, uint16_t subjectId);
unsigned long encodeCyphalServiceFrameId(uint8_t priority, uint8_t nodeId, uint16_t destinationId, uint16_t serviceId);
uint8_t trailingByteSingleFrame();
void encodeNewNodeId(unsigned char *buffer, int nodeId);
void encodeThrottleValues(uint16_t *throt, uint8_t *throtOut);

// Decoding functions
void decodeCyphalFrameId(uint32_t input, uint8_t &nodeId, uint16_t &subjectId);
void decodeBufferCurrentInfo(const unsigned char *buffer, int &electricalSpeed, int &busCurrent, uint16_t &operatingStatus);
void decodeBufferVoltageInfo(const unsigned char *buffer, int &outputThrottle, float &busVoltage, int &mosTemp, int &capacitanceTemp, int &motorTemp);

#endif // MADCYPHAL_H