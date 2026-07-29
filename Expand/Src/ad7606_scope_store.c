#include "ad7606_scope_store.h"
#include "ad9220.h"

#include "stm32h7xx_hal.h"

#include <stddef.h>
#include <string.h>

/*
 * Bank 2, sector 7 is reserved for append-only scope settings. Each update
 * consumes one 64-byte record, giving 2048 saves before a sector erase.
 */
#define SCOPE_STORE_FLASH_START       0x081E0000UL
#define SCOPE_STORE_FLASH_SIZE        0x00020000UL
#define SCOPE_STORE_RECORD_MAGIC      0x53434F50UL
#define SCOPE_STORE_RECORD_VERSION    3UL
#define SCOPE_STORE_RECORD_SIZE       64U
#define SCOPE_STORE_RECORD_COUNT      \
  (SCOPE_STORE_FLASH_SIZE / SCOPE_STORE_RECORD_SIZE)
#define SCOPE_STORE_LEGACY_RATE_HZ    2000000U

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t sequence;
  uint32_t crc32;
  uint32_t sample_rate_hz;
  int32_t center_mv;
  uint32_t time_per_div_us;
  uint32_t refresh_ms;
  uint16_t mv_per_div;
  uint16_t decimation;
  uint8_t channel;
  uint8_t running;
  uint8_t center_auto;
  uint8_t stream_enabled;
  uint32_t reserved[6];
} ScopeStoreRecord;

_Static_assert(sizeof(ScopeStoreRecord) == SCOPE_STORE_RECORD_SIZE,
               "ScopeStoreRecord must occupy two H7 flash words");

static ScopeStoreRecord scope_store_write_record
    __attribute__((aligned(32)));

static uint32_t ScopeStoreCrc32(const ScopeStoreRecord *record);
static uint8_t ScopeStoreRecordIsValid(const ScopeStoreRecord *record);
static uint8_t ScopeStoreRecordIsBlank(const ScopeStoreRecord *record);
static uint8_t ScopeStorePayloadMatches(const ScopeStoreRecord *record,
                                        const AD7606_ScopeConfig *config,
                                        uint32_t sample_rate_hz,
                                        uint8_t stream_enabled);
static uint8_t ScopeStoreConfigValuesAreValid(
    const AD7606_ScopeConfig *config, uint32_t sample_rate_hz,
    uint8_t stream_enabled);
static void ScopeStoreFindLatest(const ScopeStoreRecord **latest,
                                 uint32_t *next_address);

uint8_t AD7606_ScopeStoreLoad(AD7606_ScopeConfig *config,
                              uint32_t *sample_rate_hz,
                              uint8_t *stream_enabled)
{
  const ScopeStoreRecord *latest;
  uint32_t unused_address;

  if ((config == NULL) || (sample_rate_hz == NULL) ||
      (stream_enabled == NULL))
  {
    return 0U;
  }

  ScopeStoreFindLatest(&latest, &unused_address);
  if (latest == NULL)
  {
    return 0U;
  }

  config->channel = latest->channel;
  config->running = latest->running;
  config->center_auto = latest->center_auto;
  config->reserved = 0U;
  config->mv_per_div = latest->mv_per_div;
  config->decimation = latest->decimation;
  config->center_mv = latest->center_mv;
  config->time_per_div_us = latest->time_per_div_us;
  config->refresh_ms = latest->refresh_ms;
  /*
   * Keep the saved display configuration when upgrading from the former
   * 2 MSPS firmware, but always report the fixed rate of the running driver.
   * Flash is not rewritten until the user explicitly sends SAVE.
   */
  *sample_rate_hz = AD9220_SAMPLE_RATE_HZ;
  *stream_enabled = latest->stream_enabled;
  return 1U;
}

uint8_t AD7606_ScopeStoreSave(const AD7606_ScopeConfig *config,
                              uint32_t sample_rate_hz,
                              uint8_t stream_enabled)
{
  const ScopeStoreRecord *latest;
  uint32_t address;
  uint32_t sequence = 1U;
  HAL_StatusTypeDef status;

  if (ScopeStoreConfigValuesAreValid(config, sample_rate_hz,
                                     stream_enabled) == 0U)
  {
    return 0U;
  }

  ScopeStoreFindLatest(&latest, &address);
  if ((latest != NULL) &&
      (ScopeStorePayloadMatches(latest, config, sample_rate_hz,
                                stream_enabled) != 0U))
  {
    return 1U;
  }
  if (latest != NULL)
  {
    sequence = latest->sequence + 1U;
  }

  if (address == 0U)
  {
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_2;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
      return 0U;
    }
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();
    if ((status != HAL_OK) || (sector_error != 0xFFFFFFFFU))
    {
      return 0U;
    }
    SCB_InvalidateDCache_by_Addr((void *)SCOPE_STORE_FLASH_START,
                                 (int32_t)SCOPE_STORE_FLASH_SIZE);
    address = SCOPE_STORE_FLASH_START;
  }

  memset(&scope_store_write_record, 0xFF,
         sizeof(scope_store_write_record));
  scope_store_write_record.magic = SCOPE_STORE_RECORD_MAGIC;
  scope_store_write_record.version = SCOPE_STORE_RECORD_VERSION;
  scope_store_write_record.sequence = sequence;
  scope_store_write_record.crc32 = 0U;
  scope_store_write_record.sample_rate_hz = sample_rate_hz;
  scope_store_write_record.center_mv = config->center_mv;
  scope_store_write_record.time_per_div_us = config->time_per_div_us;
  scope_store_write_record.refresh_ms = config->refresh_ms;
  scope_store_write_record.mv_per_div = config->mv_per_div;
  scope_store_write_record.decimation = config->decimation;
  scope_store_write_record.channel = config->channel;
  scope_store_write_record.running = config->running;
  scope_store_write_record.center_auto = config->center_auto;
  scope_store_write_record.stream_enabled =
      (stream_enabled != 0U) ? 1U : 0U;
  scope_store_write_record.crc32 =
      ScopeStoreCrc32(&scope_store_write_record);

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return 0U;
  }
  status = HAL_FLASH_Program(
      FLASH_TYPEPROGRAM_FLASHWORD, address,
      (uint32_t)(uintptr_t)&scope_store_write_record);
  if (status == HAL_OK)
  {
    status = HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_FLASHWORD, address + 32U,
        (uint32_t)(uintptr_t)((const uint8_t *)&scope_store_write_record +
                             32U));
  }
  (void)HAL_FLASH_Lock();

  SCB_InvalidateDCache_by_Addr((void *)address,
                               (int32_t)sizeof(ScopeStoreRecord));
  if ((status != HAL_OK) ||
      (ScopeStoreRecordIsValid((const ScopeStoreRecord *)address) == 0U))
  {
    return 0U;
  }
  return 1U;
}

static uint32_t ScopeStoreCrc32(const ScopeStoreRecord *record)
{
  const uint8_t *bytes = (const uint8_t *)record;
  uint32_t crc = 0xFFFFFFFFU;

  for (uint32_t i = 0U; i < sizeof(*record); ++i)
  {
    uint8_t value = ((i >= offsetof(ScopeStoreRecord, crc32)) &&
                     (i < (offsetof(ScopeStoreRecord, crc32) +
                           sizeof(record->crc32)))) ? 0U : bytes[i];
    crc ^= value;
    for (uint32_t bit = 0U; bit < 8U; ++bit)
    {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

static uint8_t ScopeStoreRecordIsValid(const ScopeStoreRecord *record)
{
  AD7606_ScopeConfig config;

  if ((record->magic != SCOPE_STORE_RECORD_MAGIC) ||
      (record->version != SCOPE_STORE_RECORD_VERSION) ||
      (record->crc32 != ScopeStoreCrc32(record)))
  {
    return 0U;
  }

  config.channel = record->channel;
  config.running = record->running;
  config.center_auto = record->center_auto;
  config.reserved = 0U;
  config.mv_per_div = record->mv_per_div;
  config.decimation = record->decimation;
  config.center_mv = record->center_mv;
  config.time_per_div_us = record->time_per_div_us;
  config.refresh_ms = record->refresh_ms;
  return ScopeStoreConfigValuesAreValid(
      &config, record->sample_rate_hz, record->stream_enabled);
}

static uint8_t ScopeStoreRecordIsBlank(const ScopeStoreRecord *record)
{
  const uint32_t *words = (const uint32_t *)record;

  for (uint32_t i = 0U; i < (sizeof(*record) / sizeof(uint32_t)); ++i)
  {
    if (words[i] != 0xFFFFFFFFU)
    {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t ScopeStorePayloadMatches(const ScopeStoreRecord *record,
                                        const AD7606_ScopeConfig *config,
                                        uint32_t sample_rate_hz,
                                        uint8_t stream_enabled)
{
  return ((record->sample_rate_hz == sample_rate_hz) &&
          (record->center_mv == config->center_mv) &&
          (record->time_per_div_us == config->time_per_div_us) &&
          (record->refresh_ms == config->refresh_ms) &&
          (record->mv_per_div == config->mv_per_div) &&
          (record->decimation == config->decimation) &&
          (record->channel == config->channel) &&
          (record->running == config->running) &&
          (record->center_auto == config->center_auto) &&
          (record->stream_enabled ==
           ((stream_enabled != 0U) ? 1U : 0U))) ? 1U : 0U;
}

static uint8_t ScopeStoreConfigValuesAreValid(
    const AD7606_ScopeConfig *config, uint32_t sample_rate_hz,
    uint8_t stream_enabled)
{
  uint8_t mv_valid;
  uint8_t dec_valid;

  if (config == NULL)
  {
    return 0U;
  }
  mv_valid = ((config->mv_per_div == 50U) ||
              (config->mv_per_div == 100U) ||
              (config->mv_per_div == 200U) ||
              (config->mv_per_div == 500U) ||
              (config->mv_per_div == 1000U) ||
              (config->mv_per_div == 2000U)) ? 1U : 0U;
  dec_valid = ((config->decimation == 1U) ||
               (config->decimation == 2U) ||
               (config->decimation == 4U) ||
               (config->decimation == 8U) ||
               (config->decimation == 16U) ||
               (config->decimation == 32U)) ? 1U : 0U;

  if ((config->channel != 1U) ||
      (config->running > 1U) || (config->center_auto > 1U) ||
      (config->center_mv < -2500) || (config->center_mv > 2500) ||
      (mv_valid == 0U) || (dec_valid == 0U) ||
      ((config->time_per_div_us != 0U) &&
       ((config->time_per_div_us < 10U) ||
        (config->time_per_div_us > 1000000U))) ||
      (config->refresh_ms < 250U) || (config->refresh_ms > 2000U) ||
      ((sample_rate_hz != AD9220_SAMPLE_RATE_HZ) &&
       (sample_rate_hz != SCOPE_STORE_LEGACY_RATE_HZ)) ||
      (stream_enabled > 1U))
  {
    return 0U;
  }
  return 1U;
}

static void ScopeStoreFindLatest(const ScopeStoreRecord **latest,
                                 uint32_t *next_address)
{
  const ScopeStoreRecord *best = NULL;
  uint32_t empty_address = 0U;

  for (uint32_t i = 0U; i < SCOPE_STORE_RECORD_COUNT; ++i)
  {
    uint32_t address =
        SCOPE_STORE_FLASH_START + (i * SCOPE_STORE_RECORD_SIZE);
    const ScopeStoreRecord *record =
        (const ScopeStoreRecord *)(uintptr_t)address;

    if (ScopeStoreRecordIsBlank(record) != 0U)
    {
      if (empty_address == 0U)
      {
        empty_address = address;
      }
    }
    else if (ScopeStoreRecordIsValid(record) != 0U)
    {
      if ((best == NULL) ||
          ((int32_t)(record->sequence - best->sequence) > 0))
      {
        best = record;
      }
    }
  }

  *latest = best;
  *next_address = empty_address;
}
