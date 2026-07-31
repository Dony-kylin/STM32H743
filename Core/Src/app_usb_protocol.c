#include "app_usb_protocol.h"

#include <string.h>

static uint16_t read_u16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void write_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

uint16_t APP_USB_ProtocolCrc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;

  if ((data == NULL) && (length != 0U))
  {
    return 0U;
  }

  for (size_t index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U) ?
          (uint16_t)((crc << 1) ^ 0x1021U) :
          (uint16_t)(crc << 1);
    }
  }
  return crc;
}

size_t APP_USB_ProtocolEncode(uint8_t type, uint16_t sequence,
                             const void *payload, uint16_t payload_length,
                             uint8_t *output, size_t capacity)
{
  size_t frame_length =
      APP_USB_PROTOCOL_HEADER_SIZE + payload_length +
      APP_USB_PROTOCOL_CRC_SIZE;

  if ((output == NULL) ||
      (payload_length > APP_USB_PROTOCOL_MAX_PAYLOAD) ||
      ((payload_length != 0U) && (payload == NULL)) ||
      (capacity < frame_length))
  {
    return 0U;
  }

  output[0] = APP_USB_PROTOCOL_MAGIC0;
  output[1] = APP_USB_PROTOCOL_MAGIC1;
  output[2] = APP_USB_PROTOCOL_VERSION;
  output[3] = type;
  write_u16(&output[4], sequence);
  write_u16(&output[6], payload_length);
  if (payload_length != 0U)
  {
    memcpy(&output[APP_USB_PROTOCOL_HEADER_SIZE], payload, payload_length);
  }
  write_u16(&output[APP_USB_PROTOCOL_HEADER_SIZE + payload_length],
            APP_USB_ProtocolCrc16(&output[2], 6U + payload_length));
  return frame_length;
}

void APP_USB_ProtocolParserInit(AppUsbProtocolParser *parser)
{
  if (parser != NULL)
  {
    memset(parser, 0, sizeof(*parser));
  }
}

static void parser_resync(AppUsbProtocolParser *parser, uint8_t byte)
{
  parser->used = 0U;
  parser->expected = 0U;
  if (byte == APP_USB_PROTOCOL_MAGIC0)
  {
    parser->data[parser->used++] = byte;
  }
}

bool APP_USB_ProtocolParserPush(AppUsbProtocolParser *parser, uint8_t byte,
                               AppUsbFrame *frame)
{
  uint16_t payload_length;
  uint16_t received_crc;
  uint16_t calculated_crc;

  if ((parser == NULL) || (frame == NULL))
  {
    return false;
  }

  if (parser->used == 0U)
  {
    parser_resync(parser, byte);
    return false;
  }

  if ((parser->used == 1U) && (byte != APP_USB_PROTOCOL_MAGIC1))
  {
    parser_resync(parser, byte);
    return false;
  }

  if (parser->used >= sizeof(parser->data))
  {
    ++parser->format_errors;
    parser_resync(parser, byte);
    return false;
  }

  parser->data[parser->used++] = byte;
  if (parser->used == APP_USB_PROTOCOL_HEADER_SIZE)
  {
    payload_length = read_u16(&parser->data[6]);
    if ((parser->data[2] != APP_USB_PROTOCOL_VERSION) ||
        (payload_length > APP_USB_PROTOCOL_MAX_PAYLOAD))
    {
      ++parser->format_errors;
      parser_resync(parser, byte);
      return false;
    }
    parser->expected =
        APP_USB_PROTOCOL_HEADER_SIZE + payload_length +
        APP_USB_PROTOCOL_CRC_SIZE;
  }

  if ((parser->expected == 0U) || (parser->used < parser->expected))
  {
    return false;
  }

  payload_length = read_u16(&parser->data[6]);
  received_crc =
      read_u16(&parser->data[APP_USB_PROTOCOL_HEADER_SIZE + payload_length]);
  calculated_crc =
      APP_USB_ProtocolCrc16(&parser->data[2], 6U + payload_length);
  if (received_crc != calculated_crc)
  {
    ++parser->crc_errors;
    parser_resync(parser, byte);
    return false;
  }

  frame->type = parser->data[3];
  frame->sequence = read_u16(&parser->data[4]);
  frame->payload_length = payload_length;
  if (payload_length != 0U)
  {
    memcpy(frame->payload,
           &parser->data[APP_USB_PROTOCOL_HEADER_SIZE],
           payload_length);
  }

  parser->used = 0U;
  parser->expected = 0U;
  return true;
}
