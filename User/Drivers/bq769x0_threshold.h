#ifndef BQ769X0_THRESHOLD_H
#define BQ769X0_THRESHOLD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    BQ769X0_TRIP_UV = 0,
    BQ769X0_TRIP_OV
} BQ769X0_TripThresholdType_t;

/*
 * Convert a requested physical threshold into the BQ769X0 middle-eight-bit
 * register format.  The full ADC code is checked before truncation so an
 * out-of-range request can never wrap into an unrelated threshold.
 */
static inline bool BQ769X0_ThresholdEncode(uint16_t requested_mv,
                                           uint16_t gain_uv_per_lsb,
                                           int8_t offset_mv,
                                           BQ769X0_TripThresholdType_t type,
                                           uint8_t *encoded,
                                           uint16_t *actual_mv)
{
    uint16_t code_min;
    uint16_t code_max;
    uint16_t code_base;
    uint16_t full_code;
    uint16_t programmed_code;
    uint32_t corrected_uv;
    uint32_t full_code_wide;
    int32_t programmed_uv;
    int32_t physical_mv;
    int32_t error_mv;
    uint16_t quantization_mv;

    if ((encoded == NULL) || (actual_mv == NULL) ||
        (gain_uv_per_lsb == 0U) ||
        ((type != BQ769X0_TRIP_UV) && (type != BQ769X0_TRIP_OV)) ||
        ((int32_t)requested_mv <= (int32_t)offset_mv))
    {
        return false;
    }

    if (type == BQ769X0_TRIP_OV)
    {
        code_min = 0x2000U;
        code_max = 0x2FFFU;
        code_base = 0x2008U;
    }
    else
    {
        code_min = 0x1000U;
        code_max = 0x1FFFU;
        code_base = 0x1000U;
    }

    corrected_uv = (uint32_t)((int32_t)requested_mv - (int32_t)offset_mv) * 1000U;
    full_code_wide = (corrected_uv + (gain_uv_per_lsb / 2U)) /
                     gain_uv_per_lsb;
    if (full_code_wide > 0xFFFFU)
    {
        return false;
    }
    full_code = (uint16_t)full_code_wide;
    if ((full_code < code_min) || (full_code > code_max))
    {
        return false;
    }

    *encoded = (uint8_t)((full_code >> 4U) & 0xFFU);
    programmed_code = (uint16_t)(code_base + ((uint16_t)(*encoded) << 4U));
    programmed_uv = ((int32_t)programmed_code * (int32_t)gain_uv_per_lsb) +
                    ((int32_t)offset_mv * 1000);
    if (programmed_uv < 0)
    {
        return false;
    }

    physical_mv = (programmed_uv + 500) / 1000;
    error_mv = physical_mv - (int32_t)requested_mv;
    if (error_mv < 0)
    {
        error_mv = -error_mv;
    }
    quantization_mv = (uint16_t)(((uint32_t)gain_uv_per_lsb * 16U + 999U) /
                                 1000U);
    if (error_mv > (int32_t)quantization_mv)
    {
        return false;
    }

    *actual_mv = (uint16_t)physical_mv;
    return true;
}

/* Reconstruct the physical threshold represented by a fresh register read. */
static inline bool BQ769X0_ThresholdDecode(uint8_t encoded,
                                           uint16_t gain_uv_per_lsb,
                                           int8_t offset_mv,
                                           BQ769X0_TripThresholdType_t type,
                                           uint16_t *actual_mv)
{
    uint16_t code_base;
    uint16_t programmed_code;
    int32_t programmed_uv;

    if ((actual_mv == NULL) || (gain_uv_per_lsb == 0U) ||
        ((type != BQ769X0_TRIP_UV) && (type != BQ769X0_TRIP_OV)))
    {
        return false;
    }

    code_base = (type == BQ769X0_TRIP_OV) ? 0x2008U : 0x1000U;
    programmed_code = (uint16_t)(code_base + ((uint16_t)encoded << 4U));
    programmed_uv = ((int32_t)programmed_code * (int32_t)gain_uv_per_lsb) +
                    ((int32_t)offset_mv * 1000);
    if (programmed_uv < 0)
    {
        return false;
    }

    *actual_mv = (uint16_t)((programmed_uv + 500) / 1000);
    return true;
}

#endif /* BQ769X0_THRESHOLD_H */
