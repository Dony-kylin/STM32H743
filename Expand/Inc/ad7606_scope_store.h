#ifndef __AD7606_SCOPE_STORE_H__
#define __AD7606_SCOPE_STORE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ad7606_scope.h"

#include <stdint.h>

uint8_t AD7606_ScopeStoreLoad(AD7606_ScopeConfig *config,
                              uint32_t *sample_rate_hz,
                              uint8_t *stream_enabled);
uint8_t AD7606_ScopeStoreSave(const AD7606_ScopeConfig *config,
                              uint32_t sample_rate_hz,
                              uint8_t stream_enabled);

#ifdef __cplusplus
}
#endif

#endif /* __AD7606_SCOPE_STORE_H__ */
