#ifndef __AD9220_H__
#define __AD9220_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define AD9220_ADC_CLOCK_HZ            8000000U
#define AD9220_SAMPLE_RATE_HZ          2000000U
#define AD9220_CLOCKS_PER_SAMPLE       4U
#define AD9220_CAPTURE_SAMPLES         16384U
#define AD9220_PIPELINE_DELAY          3U
#define AD9220_MAX_CAPTURE_SAMPLES     AD9220_CAPTURE_SAMPLES

typedef enum
{
  AD9220_STATUS_OK = 0,
  AD9220_STATUS_BUSY,
  AD9220_STATUS_INVALID_ARGUMENT,
  AD9220_STATUS_ERROR
} AD9220_Status;

/*
 * Pin assignment:
 *   D0  PD1    D1  PE8    D2  PE10   D3  PE12
 *   D4  PE14   D5  PD8    D6  PE15   D7  PE13
 *   D8  PE11   D9  PE9    D10 PE7    D11 PD0
 *   D12/OTR PD14   CLK PD15 (direct GPIO input)
 *   The external TCXO drives only AD9220 CLK and PD15.
 *
 * The first point is synchronized to the external 8 MHz clock. A DWT-paced
 * polling loop then reads GPIOE/GPIOD every 240 CPU cycles, giving a precise
 * 2 MHz sample rate. Each port pair is checked twice for consistency.
 */
void AD9220_Init(void);
void AD9220_DeInit(void);

AD9220_Status AD9220_StartCapture(uint32_t sample_count);
void AD9220_AbortCapture(void);
uint8_t AD9220_IsCaptureBusy(void);
uint8_t AD9220_IsCaptureComplete(void);

uint32_t AD9220_GetSampleRateHz(void);
uint32_t AD9220_GetCapturedCount(void);
uint32_t AD9220_GetDmaErrorCount(void);
uint32_t AD9220_GetOverrangeCount(void);
uint32_t AD9220_GetDmaDoneMask(void);
uint32_t AD9220_GetPortEDmaErrorCode(void);
uint32_t AD9220_GetPortDDmaErrorCode(void);
uint32_t AD9220_GetLastTimerDelta(void);
uint32_t AD9220_GetTimerDeltaLimit(void);
uint32_t AD9220_GetGlitchCorrectionCount(void);
uint32_t AD9220_GetLastProgress(void);
uint32_t AD9220_GetLastErrorStage(void);
uint32_t AD9220_GetLastClockLevel(void);

uint16_t AD9220_GetRawSample(uint32_t index);
int16_t AD9220_GetSignedSample(uint32_t index);
uint8_t AD9220_GetOverrange(uint32_t index);
uint32_t AD9220_CopySignedSamples(int16_t *destination,
                                  uint32_t capacity);

void AD9220_DMA_PortE_IRQHandler(void);
void AD9220_DMA_PortD_IRQHandler(void);

/* Called after a capture completes or an edge-wait error is detected. */
void AD9220_CaptureCompleteCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD9220_H__ */
