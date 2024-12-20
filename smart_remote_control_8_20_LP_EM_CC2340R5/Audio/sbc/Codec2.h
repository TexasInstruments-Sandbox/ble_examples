/*
 * Codec2.h
 *
 *  Created on: Sep 30, 2023
 *      Author: a0132664
 */

#ifndef APPLICATION_SBC_CODEC2_H_
#define APPLICATION_SBC_CODEC2_H_

#if (defined AUDIO_RECEIVER) || (defined AUDIO_TRANSMITTER)
extern void Codec2_init(void);
#endif //AUDIO_RECEIVER || AUDIO_TRANSMITTER

#ifdef AUDIO_RECEIVER
extern void Codec2_decodeBuff(int16_t* dst, uint8_t* src, unsigned dstSize, unsigned srcSize);
#endif //AUDIO_RECEIVER

#ifdef AUDIO_TRANSMITTER
extern uint8_t Codec2_encodeBuff(uint8_t* dst, int16_t* src, unsigned dstSize, unsigned srcSize);
#endif //AUDIO_TRANSMITTER

#endif /* APPLICATION_SBC_CODEC2_H_ */
