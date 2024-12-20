/*
 * Codec2.c
 *
 *  Created on: Sep 30, 2023
 *      Author: a0132664
 */

#include <Audio/sbc/include/msbc_library.h>

sbc_t sbc;
size_t written;

#if (defined AUDIO_RECEIVER) || (defined AUDIO_TRANSMITTER)
void Codec2_init(void)
{
    sbc_init_msbc(&sbc, 0);
    written = 0;
}
#endif //AUDIO_RECEIVER || AUDIO_TRANSMITTER

#ifdef AUDIO_RECEIVER
void Codec2_decodeBuff(int16_t* dst, uint8_t* src, unsigned dstSize, unsigned srcSize)
{
    sbc_decode(&sbc,                                   /* State structure for MSBC encoder */
               src,                                    /* Encoded bitstream */
               srcSize,                                /* Size of encoded bistream in bytes */
               dst,                                    /* Buffer to store decoded PCM data */
               dstSize*sizeof(int16_t),                /* Size of PCM frame in bytes */
               &written);                              /* Number of bytes written by codec */
}
#endif //AUDIO_RECEIVER

#ifdef AUDIO_TRANSMITTER
uint8_t Codec2_encodeBuff(uint8_t* dst, int16_t* src, unsigned dstSize, unsigned srcSize)
{
    return (uint8_t) sbc_encode(&sbc,                   /* State structure for MSBC */
                                src,                    /* Unencoded PCM data */
                                srcSize*sizeof(int16_t),/* Size of PCM buffer in bytes */
                                dst,                    /* Buffer to store encoded data */
                                dstSize,                /* Size of encoded frame **fixed for MSBC** */
                                &written);              /* Number of bytes written */
}
#endif //AUDIO_TRANSMITTER
