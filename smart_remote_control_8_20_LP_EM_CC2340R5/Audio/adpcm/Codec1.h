/*
 * Copyright (c) 2016-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @mainpage IMA Adaptive Differential Pulse Code Modulation (ADPCM) Codec
 *
 *  This implementation implements IMA-ADPCM 4:1 compression/decompression.
 *  The algorithm implemented is based on the one described in section 6 of
 *  [Recommended Practices for Enhancing Digital Audio Compatibility in Multimedia Systems](http://www.cs.columbia.edu/~hgs/audio/dvi/IMA_ADPCM.pdf).
 *
 *  @warning ADPCM supports 16-bit samples only.
 *
 *  See the @ref adpcm  for information about usage.
 *
 * @defgroup adpcm ADPCM API
 * @{
 */

#ifndef ti_adpcm_Codec1__include
#define ti_adpcm_Codec1__include

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief  This routine encodes one int16 sample with TI Codec Type 1.
 *
 *  @param  audSample  The audtion sample to encode
 *
 *  @return The encoded result as a 4-bit nibble
 */
extern uint8_t Codec1_encodeSingle(int16_t audSample);


/*!
 *  @brief  This routine decode a 4-bit nibble sample to a uint16 PCM audio sample.
 *
 *  @param  nibble_4bits  A 4-bit nibble to decode
 *
 *  @return The decoded value as a 16-bit PCM sample
 */
extern int16_t Codec1_decodeSingle(uint8_t nibble_4bits);

/*!
 *  @brief  This routine encode a buffer with TI codec Type 1.
 *
 *  @param  dst Pointer to the buffer where encoding result will be written to
 *
 *  @param  src Pointer to the buffer that should be encoded.
 *              Must be a multiple of 4 bytes
 *
 *  @param  srcSize The number of samples (int16) in the src buffer.
 *                  Must be a multiple of 2.
 *
 *  @param  si Pointer to the current step index
 *
 *  @param  pv Pointer to the current predicted-value
 *
 *  @return Number of bytes written to the destination buffer
 */
extern uint8_t Codec1_encodeBuff(uint8_t* dst, int16_t* src, int16_t srcSize, int8_t *si, int16_t *pv);

/*!
 *  @brief  This routine decodes a buffer with TI codec Type 1.
 *
 *  @param  dst Pointer to the buffer where decoded result will be written to
 *
 *  @param  src Pointer to the buffer that should be decoded.
 *              Must be a multiple of 4 bytes
 *
 *  @param  srcSize Number of byte that will be generated by
 *                  the encoder (4* (src buffer size in byte))
 *
 *  @param  si Pointer to the current step index
 *
 *  @param  pv Pointer to the current predicted-value
 */
extern void Codec1_decodeBuff(int16_t* dst, uint8_t* src, unsigned srcSize,  int8_t *si, int16_t *pv);

#ifdef __cplusplus
}
#endif

#endif /* ti_adpcm_Codec1__include */
/*! @}*/