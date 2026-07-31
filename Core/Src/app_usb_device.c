#include "app_usb_device.h"

#include "main.h"
#include "usbd_cdc_if.h"

#define APP_USB_RX_RING_SIZE       1024U
#define APP_USB_RX_RING_MASK       (APP_USB_RX_RING_SIZE - 1U)
#define APP_USB_CONTROL_DEPTH      4U
#define APP_USB_TX_BUSY_RETRY_MS   1U
#define APP_USB_TX_ERROR_RETRY_MS  10U
#define APP_USB_RESULT_FORMAT      1U
#define APP_USB_RX_PROCESS_BUDGET  256U

#if ((APP_USB_RX_RING_SIZE & (APP_USB_RX_RING_SIZE - 1U)) != 0U)
#error "APP_USB_RX_RING_SIZE must be a power of two"
#endif

typedef struct
{
  uint16_t length;
  uint8_t data[APP_USB_PROTOCOL_MAX_FRAME_SIZE];
} AppUsbTxFrame;

typedef enum
{
  APP_USB_TX_NONE = 0U,
  APP_USB_TX_CONTROL,
  APP_USB_TX_RESULT
} AppUsbTxSource;

static volatile uint16_t rx_write;
static volatile uint16_t rx_read;
static uint8_t rx_ring[APP_USB_RX_RING_SIZE];
static volatile uint32_t rx_overflow_bytes;

static AppUsbProtocolParser parser;
static AppUsbFrame received_frame;

static AppUsbTxFrame control_queue[APP_USB_CONTROL_DEPTH];
static uint8_t control_write;
static uint8_t control_read;
static uint8_t control_count;
static uint32_t control_queue_drops;

static AppUsbTxFrame result_buffer[2];
static uint8_t result_pending_index;
static uint8_t result_pending_valid;
static uint8_t result_next_index;
static uint8_t result_inflight_index;
static uint32_t result_overwrites;
static uint32_t last_analysis_sequence;

static volatile uint8_t tx_complete;
static uint8_t tx_busy;
static AppUsbTxSource tx_source;
static uint32_t tx_busy_retries;
static uint32_t tx_errors;
static uint32_t next_tx_retry_tick;
static uint8_t link_was_ready;
static uint8_t stream_enabled;

static void write_u32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static bool time_reached(uint32_t now, uint32_t target)
{
  return ((int32_t)(now - target) >= 0);
}

void APP_USB_DeviceInit(void)
{
  rx_write = 0U;
  rx_read = 0U;
  rx_overflow_bytes = 0U;
  control_write = 0U;
  control_read = 0U;
  control_count = 0U;
  control_queue_drops = 0U;
  result_pending_index = 0U;
  result_pending_valid = 0U;
  result_next_index = 0U;
  result_inflight_index = 0U;
  result_overwrites = 0U;
  last_analysis_sequence = 0U;
  tx_complete = 0U;
  tx_busy = 0U;
  tx_source = APP_USB_TX_NONE;
  tx_busy_retries = 0U;
  tx_errors = 0U;
  next_tx_retry_tick = 0U;
  link_was_ready = 0U;
  stream_enabled = 1U;
  APP_USB_ProtocolParserInit(&parser);
}

void APP_USB_DeviceReceiveFromIsr(const uint8_t *data, uint16_t length)
{
  uint16_t index;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    uint16_t next = (uint16_t)((rx_write + 1U) & APP_USB_RX_RING_MASK);
    if (next == rx_read)
    {
      rx_overflow_bytes += (uint32_t)(length - index);
      break;
    }
    rx_ring[rx_write] = data[index];
    __DMB();
    rx_write = next;
  }
}

void APP_USB_DeviceTxCompleteFromIsr(void)
{
  tx_complete = 1U;
  __DMB();
}

static bool queue_control_frame(uint8_t type, uint16_t sequence,
                                const void *payload, uint16_t payload_length)
{
  AppUsbTxFrame *slot;
  size_t encoded_length;

  if (control_count >= APP_USB_CONTROL_DEPTH)
  {
    ++control_queue_drops;
    return false;
  }

  slot = &control_queue[control_write];
  encoded_length =
      APP_USB_ProtocolEncode(type, sequence, payload, payload_length,
                             slot->data, sizeof(slot->data));
  if (encoded_length == 0U)
  {
    ++control_queue_drops;
    return false;
  }

  slot->length = (uint16_t)encoded_length;
  control_write =
      (uint8_t)((control_write + 1U) % APP_USB_CONTROL_DEPTH);
  ++control_count;
  return true;
}

static void send_ack(const AppUsbFrame *request, AppUsbAckCode code)
{
  uint8_t payload[2];
  uint8_t response_type;

  payload[0] = request->type;
  payload[1] = (uint8_t)code;
  response_type =
      (code == APP_USB_ACK_OK) ? APP_USB_MSG_ACK : APP_USB_MSG_ERROR;
  (void)queue_control_frame(response_type, request->sequence,
                            payload, sizeof(payload));
}

static void send_status(uint16_t sequence)
{
  uint8_t payload[36];

  payload[0] = 1U;
  payload[1] = stream_enabled;
  payload[2] = CDC_IsReady_FS();
  payload[3] = tx_busy;
  write_u32(&payload[4], rx_overflow_bytes);
  write_u32(&payload[8], parser.crc_errors);
  write_u32(&payload[12], parser.format_errors);
  write_u32(&payload[16], control_queue_drops);
  write_u32(&payload[20], tx_busy_retries);
  write_u32(&payload[24], tx_errors);
  write_u32(&payload[28], result_overwrites);
  write_u32(&payload[32], last_analysis_sequence);
  (void)queue_control_frame(APP_USB_MSG_STATUS, sequence,
                            payload, sizeof(payload));
}

static void dispatch_frame(const AppUsbFrame *request)
{
  switch (request->type)
  {
    case APP_USB_MSG_START_STREAM:
      if (request->payload_length != 0U)
      {
        send_ack(request, APP_USB_ACK_BAD_LENGTH);
      }
      else
      {
        stream_enabled = 1U;
        send_ack(request, APP_USB_ACK_OK);
      }
      break;

    case APP_USB_MSG_STOP_STREAM:
      if (request->payload_length != 0U)
      {
        send_ack(request, APP_USB_ACK_BAD_LENGTH);
      }
      else
      {
        stream_enabled = 0U;
        result_pending_valid = 0U;
        send_ack(request, APP_USB_ACK_OK);
      }
      break;

    case APP_USB_MSG_GET_STATUS:
      if (request->payload_length != 0U)
      {
        send_ack(request, APP_USB_ACK_BAD_LENGTH);
      }
      else
      {
        send_status(request->sequence);
      }
      break;

    default:
      send_ack(request, APP_USB_ACK_UNSUPPORTED);
      break;
  }
}

static uint16_t encode_analysis_payload(
    const AppUsbAnalysisResult *result, uint8_t *payload, size_t capacity)
{
  uint16_t offset = 0U;
  uint8_t component_count;

  if ((result == NULL) || (payload == NULL) || (capacity < 104U))
  {
    return 0U;
  }

  component_count = result->component_count;
  if (component_count > APP_USB_ANALYSIS_COMPONENTS)
  {
    component_count = APP_USB_ANALYSIS_COMPONENTS;
  }

  payload[offset++] = APP_USB_RESULT_FORMAT;
  payload[offset++] = result->mode;
  payload[offset++] = result->periods;
  payload[offset++] = component_count;

#define APP_USB_PUT_U32(value_)                    \
  do                                               \
  {                                                \
    write_u32(&payload[offset], (value_));          \
    offset = (uint16_t)(offset + sizeof(uint32_t)); \
  } while (0)

  APP_USB_PUT_U32(result->analysis_sequence);
  APP_USB_PUT_U32(result->timestamp_ms);
  APP_USB_PUT_U32(result->status_flags);
  APP_USB_PUT_U32(result->adc_sample_rate_hz);
  APP_USB_PUT_U32(result->processing_sample_rate_hz);
  APP_USB_PUT_U32(result->input_sample_count);
  APP_USB_PUT_U32(result->fft_size);
  APP_USB_PUT_U32(result->fundamental_millihz);
  APP_USB_PUT_U32(result->vpp_uv);
  APP_USB_PUT_U32(result->vrms_uv);
  APP_USB_PUT_U32(result->thd_ppm);
  APP_USB_PUT_U32(result->bad_sample_count);
  APP_USB_PUT_U32(result->analysis_time_us);

  for (uint8_t index = 0U; index < APP_USB_ANALYSIS_COMPONENTS; ++index)
  {
    APP_USB_PUT_U32(result->component[index].harmonic_order);
    APP_USB_PUT_U32(result->component[index].frequency_millihz);
    APP_USB_PUT_U32(result->component[index].measured_amplitude_uvpk);
    APP_USB_PUT_U32(result->component[index].setting_amplitude_uvpk);
  }

#undef APP_USB_PUT_U32

  return offset;
}

bool APP_USB_DevicePublishAnalysis(const AppUsbAnalysisResult *result)
{
  uint8_t payload[APP_USB_PROTOCOL_MAX_PAYLOAD];
  uint16_t payload_length;
  uint8_t write_index;
  size_t frame_length;

  if ((result == NULL) || (stream_enabled == 0U))
  {
    return false;
  }

  payload_length =
      encode_analysis_payload(result, payload, sizeof(payload));
  if (payload_length == 0U)
  {
    return false;
  }

  if (result_pending_valid != 0U)
  {
    write_index = result_pending_index;
    ++result_overwrites;
  }
  else if ((tx_busy != 0U) && (tx_source == APP_USB_TX_RESULT))
  {
    write_index = (uint8_t)(result_inflight_index ^ 1U);
  }
  else
  {
    write_index = result_next_index;
    result_next_index ^= 1U;
  }

  frame_length = APP_USB_ProtocolEncode(
      APP_USB_MSG_ANALYSIS,
      (uint16_t)result->analysis_sequence,
      payload, payload_length,
      result_buffer[write_index].data,
      sizeof(result_buffer[write_index].data));
  if (frame_length == 0U)
  {
    return false;
  }

  result_buffer[write_index].length = (uint16_t)frame_length;
  result_pending_index = write_index;
  result_pending_valid = 1U;
  last_analysis_sequence = result->analysis_sequence;
  return true;
}

static void finish_completed_transfer(void)
{
  if (tx_complete == 0U)
  {
    return;
  }

  tx_complete = 0U;
  if (tx_busy == 0U)
  {
    return;
  }

  if ((tx_source == APP_USB_TX_CONTROL) && (control_count != 0U))
  {
    control_read =
        (uint8_t)((control_read + 1U) % APP_USB_CONTROL_DEPTH);
    --control_count;
  }

  tx_busy = 0U;
  tx_source = APP_USB_TX_NONE;
}

static void reset_tx_after_disconnect(void)
{
  tx_busy = 0U;
  tx_complete = 0U;
  tx_source = APP_USB_TX_NONE;
  control_read = 0U;
  control_write = 0U;
  control_count = 0U;
  next_tx_retry_tick = HAL_GetTick();
}

static void service_transmitter(void)
{
  uint8_t link_ready = CDC_IsReady_FS();
  uint8_t *data;
  uint16_t length;
  AppUsbTxSource source;
  uint8_t result_index = 0U;
  uint8_t status;
  uint32_t now;

  if (link_ready == 0U)
  {
    if (link_was_ready != 0U)
    {
      reset_tx_after_disconnect();
    }
    link_was_ready = 0U;
    return;
  }

  if (link_was_ready == 0U)
  {
    reset_tx_after_disconnect();
    link_was_ready = 1U;
  }

  finish_completed_transfer();
  if (tx_busy != 0U)
  {
    return;
  }

  now = HAL_GetTick();
  if (!time_reached(now, next_tx_retry_tick))
  {
    return;
  }

  if (control_count != 0U)
  {
    data = control_queue[control_read].data;
    length = control_queue[control_read].length;
    source = APP_USB_TX_CONTROL;
  }
  else if (result_pending_valid != 0U)
  {
    result_index = result_pending_index;
    data = result_buffer[result_index].data;
    length = result_buffer[result_index].length;
    source = APP_USB_TX_RESULT;
  }
  else
  {
    return;
  }

  tx_busy = 1U;
  tx_source = source;
  result_inflight_index = result_index;
  status = CDC_Transmit_FS(data, length);
  if (status == USBD_OK)
  {
    if (source == APP_USB_TX_RESULT)
    {
      result_pending_valid = 0U;
    }
    return;
  }

  tx_busy = 0U;
  tx_source = APP_USB_TX_NONE;
  if (status == USBD_BUSY)
  {
    ++tx_busy_retries;
    next_tx_retry_tick = now + APP_USB_TX_BUSY_RETRY_MS;
  }
  else
  {
    ++tx_errors;
    next_tx_retry_tick = now + APP_USB_TX_ERROR_RETRY_MS;
  }
}

void APP_USB_DeviceProcess(void)
{
  uint32_t processed = 0U;

  while ((rx_read != rx_write) &&
         (processed < APP_USB_RX_PROCESS_BUDGET))
  {
    uint8_t byte = rx_ring[rx_read];
    rx_read = (uint16_t)((rx_read + 1U) & APP_USB_RX_RING_MASK);
    ++processed;
    if (APP_USB_ProtocolParserPush(&parser, byte, &received_frame))
    {
      dispatch_frame(&received_frame);
    }
  }

  service_transmitter();
}

uint8_t APP_USB_DeviceStreamEnabled(void)
{
  return stream_enabled;
}

void APP_USB_DeviceGetStats(AppUsbDeviceStats *stats)
{
  if (stats == NULL)
  {
    return;
  }

  stats->rx_overflow_bytes = rx_overflow_bytes;
  stats->crc_errors = parser.crc_errors;
  stats->format_errors = parser.format_errors;
  stats->control_queue_drops = control_queue_drops;
  stats->tx_busy_retries = tx_busy_retries;
  stats->tx_errors = tx_errors;
  stats->result_overwrites = result_overwrites;
  stats->last_analysis_sequence = last_analysis_sequence;
  stats->stream_enabled = stream_enabled;
  stats->link_ready = CDC_IsReady_FS();
  stats->tx_busy = tx_busy;
}
