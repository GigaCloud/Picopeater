#ifndef AX25_H
#define AX25_H

#include <stdint.h>

void fillSinLUT(void);
void runCorrelator(int16_t samples[], uint16_t num_samples, uint16_t sample_rate);
uint16_t getBits(uint8_t bitbuff[], uint16_t samples[], uint16_t num_samples, uint16_t sample_rate);
void reverseNRZI(uint8_t bitbuff[], uint16_t numbits);
uint16_t getdatastartidx(uint8_t bitbuff[], uint16_t numbits, uint8_t flagcount);
uint16_t getdataendidx(uint8_t bitbuff[], uint16_t numbits, uint16_t datastartidx);
uint16_t removebitstuffing(uint8_t bitbuff[], uint16_t datastartidx, uint16_t dataendidx);
uint16_t calc_crc (uint8_t bitbuff[], uint16_t datastartidx, uint16_t dataendidx);
uint8_t getbyte(uint8_t buff[]);


#endif