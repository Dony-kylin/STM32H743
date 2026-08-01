#ifndef APP_USB_DEVICE_H
#define APP_USB_DEVICE_H

#include "app_usb_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_USB_ANALYSIS_COMPONENTS 3U

#define APP_USB_RESULT_FLAG_VALID      (1UL << 0)
#define APP_USB_RESULT_FLAG_OVERRANGE  (1UL << 1)

typedef struct
{
  uint32_t harmonic_order;
  uint32_t frequency_millihz;
  uint32_t measured_amplitude_uvpk;
  uint32_t setting_amplitude_uvpk;
} AppUsbAnalysisComponent;

typedef struct
{
  uint32_t analysis_sequence;
  uint32_t timestamp_ms;
  uint32_t status_flags;
  uint32_t adc_sample_rate_hz;
  uint32_t processing_sample_rate_hz;
  uint32_t input_sample_count;
  uint32_t fft_size;
  uint32_t fundamental_millihz;
  /* UTG2062X front-panel-equivalent Vpp after the two-range calibration. */
  uint32_t vpp_uv;
  uint32_t vrms_uv;
  uint32_t thd_ppm;
  uint32_t bad_sample_count;
  uint32_t analysis_time_us;
  uint8_t mode;
  uint8_t periods;
  uint8_t component_count;
  AppUsbAnalysisComponent component[APP_USB_ANALYSIS_COMPONENTS];
} AppUsbAnalysisResult;

typedef struct
{
  uint32_t rx_overflow_bytes;
  uint32_t crc_errors;
  uint32_t format_errors;
  uint32_t control_queue_drops;
  uint32_t tx_busy_retries;
  uint32_t tx_errors;
  uint32_t result_overwrites;
  uint32_t last_analysis_sequence;
  uint8_t stream_enabled;
  uint8_t link_ready;
  uint8_t tx_busy;
} AppUsbDeviceStats;

void APP_USB_DeviceInit(void);
void APP_USB_DeviceProcess(void);
void APP_USB_DeviceReceiveFromIsr(const uint8_t *data, uint16_t length);
void APP_USB_DeviceTxCompleteFromIsr(void);
bool APP_USB_DevicePublishAnalysis(const AppUsbAnalysisResult *result);
uint8_t APP_USB_DeviceStreamEnabled(void);
void APP_USB_DeviceGetStats(AppUsbDeviceStats *stats);

#endif
