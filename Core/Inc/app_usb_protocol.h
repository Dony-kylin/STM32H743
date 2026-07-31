#ifndef APP_USB_PROTOCOL_H
#define APP_USB_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_USB_PROTOCOL_MAGIC0          0xA5U
#define APP_USB_PROTOCOL_MAGIC1          0x5AU
#define APP_USB_PROTOCOL_VERSION         0x01U
#define APP_USB_PROTOCOL_MAX_PAYLOAD     128U
#define APP_USB_PROTOCOL_HEADER_SIZE     8U
#define APP_USB_PROTOCOL_CRC_SIZE        2U
#define APP_USB_PROTOCOL_MAX_FRAME_SIZE  \
  (APP_USB_PROTOCOL_HEADER_SIZE + APP_USB_PROTOCOL_MAX_PAYLOAD + \
   APP_USB_PROTOCOL_CRC_SIZE)

typedef enum
{
  APP_USB_MSG_START_STREAM = 0x01U,
  APP_USB_MSG_STOP_STREAM  = 0x02U,
  APP_USB_MSG_GET_STATUS   = 0x11U,
  APP_USB_MSG_ACK          = 0x70U,
  APP_USB_MSG_ERROR        = 0x71U,
  APP_USB_MSG_STATUS       = 0x72U,
  APP_USB_MSG_ANALYSIS     = 0x80U
} AppUsbMessageType;

typedef enum
{
  APP_USB_ACK_OK = 0U,
  APP_USB_ACK_BAD_LENGTH = 1U,
  APP_USB_ACK_UNSUPPORTED = 2U,
  APP_USB_ACK_BUSY = 3U
} AppUsbAckCode;

typedef struct
{
  uint8_t type;
  uint16_t sequence;
  uint16_t payload_length;
  uint8_t payload[APP_USB_PROTOCOL_MAX_PAYLOAD];
} AppUsbFrame;

typedef struct
{
  uint8_t data[APP_USB_PROTOCOL_MAX_FRAME_SIZE];
  uint16_t used;
  uint16_t expected;
  uint32_t crc_errors;
  uint32_t format_errors;
} AppUsbProtocolParser;

uint16_t APP_USB_ProtocolCrc16(const uint8_t *data, size_t length);
size_t APP_USB_ProtocolEncode(uint8_t type, uint16_t sequence,
                             const void *payload, uint16_t payload_length,
                             uint8_t *output, size_t output_capacity);
void APP_USB_ProtocolParserInit(AppUsbProtocolParser *parser);
bool APP_USB_ProtocolParserPush(AppUsbProtocolParser *parser, uint8_t byte,
                               AppUsbFrame *frame);

#endif
