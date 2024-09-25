#include "ax25.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "Picopeater.h"

#define SINAMPL (1 << 8)




//These are hardcoded because I don't want to link the math library, but they are calculated with the functions below
/**
int16_t sinLUT_MARK[MARK_SIZE];
int16_t sinLUT_SPACE[SPACE_SIZE];
int16_t cosLUT_MARK[MARK_SIZE];
int16_t cosLUT_SPACE[SPACE_SIZE];
*/
//these value and sizes work for a fixed sampling freq of 39600
int16_t sinLUT_MARK[MARK_SIZE] = {0x0, 0x30, 0x5f, 0x8a, 0xb0, 0xd0, 0xe8, 0xf8, 0xff, 0xfd, 0xf1, 0xdd, 0xc1, 0x9e, 0x75, 0x48, 0x18, 0xe8, 0xb8, 0x8b, 0x62, 0x3f, 0x23, 0x0f, 0x03, 0x01, 0x08, 0x18, 0x30, 0x50, 0x76, 0xa1, 0xd0};
int16_t sinLUT_SPACE[SPACE_SIZE] = {0x0, 0x57, 0xa4, 0xdd, 0xfc, 0xfc, 0xdd, 0xa4, 0x57, 0x0, 0xa9, 0x5c, 0x23, 0x04, 0x04, 0x23, 0x5c, 0xa9 };
int16_t cosLUT_MARK[MARK_SIZE] = {0x100, 0xfb, 0xed, 0xd7, 0xb9, 0x94, 0x6a, 0x3c, 0xc, 0xdc, 0xad, 0x81, 0x59, 0x37, 0x1d, 0x0b, 0x02, 0x02, 0x0b, 0x1d, 0x37, 0x59, 0x80, 0xad, 0xdc, 0xc, 0x3c, 0x6a, 0x94, 0xb9, 0xd7, 0xed, 0xfb};
int16_t cosLUT_SPACE[SPACE_SIZE] = {0x100, 0xf0, 0xc4, 0x80, 0x2c, 0xd4, 0x81, 0x3c, 0x10, 0x00, 0x10, 0x3c, 0x80, 0xd4, 0x2c, 0x7f, 0xc4, 0xf0 };


/*
void fillSinLUT(void){
    for(int i = 0; i < MARK_SIZE; ++i) {
        sinLUT_MARK[i] = SINAMPL * sin(2 * M_PI * i * ((double)MARK_FREQ/SAMPLE_RATE));
        cosLUT_MARK[i] = SINAMPL * cos(2 * M_PI * i * ((double)MARK_FREQ/SAMPLE_RATE));
    }
    for(int i = 0; i < SPACE_SIZE; ++i) {
        sinLUT_SPACE[i] = SINAMPL * sin(2 * M_PI * i * ((double)SPACE_FREQ/SAMPLE_RATE));
        cosLUT_SPACE[i] = SINAMPL * cos(2 * M_PI * i * ((double)SPACE_FREQ/SAMPLE_RATE));
    }
} */

static int16_t getSinCorr(uint16_t sampleidx, uint16_t freq){
    switch(freq){
        case MARK_FREQ:
            return sinLUT_MARK[sampleidx % MARK_SIZE];
        break;

        case SPACE_FREQ:
            return sinLUT_SPACE[sampleidx % SPACE_SIZE];
        break;
    }
}

static int16_t getCosCorr(uint16_t sampleidx, uint16_t freq){
    switch(freq){
        case MARK_FREQ:
            return cosLUT_MARK[sampleidx % MARK_SIZE];
        break;

        case SPACE_FREQ:
            return cosLUT_SPACE[sampleidx % SPACE_SIZE];
        break;
    }
}



void runCorrelator(int16_t samples[], uint16_t num_samples, uint16_t sample_rate){
    uint16_t window_size = sample_rate / APRS_BAUD;

    for(size_t i = 0; i < num_samples - window_size - 1; ++i){
        int32_t markI = 0;
        int32_t markQ = 0;
        int32_t spaceI = 0;
        int32_t spaceQ = 0;

        for(size_t j = i; j < i + window_size; ++j){
            markI += samples[j] * getSinCorr(j, MARK_FREQ);
            markQ += samples[j] * getCosCorr(j, MARK_FREQ);
            spaceI += samples[j] * getSinCorr(j, SPACE_FREQ);
            spaceQ += samples[j] * getCosCorr(j, SPACE_FREQ);
        }

        int32_t markSquare = ABS(markI) + ABS(markQ);
        int32_t spaceSquare = ABS(spaceI) + ABS(spaceQ);

        samples[i] = markSquare > spaceSquare ? 1 : 0;
    }
}

uint16_t getBits(uint8_t bitbuff[], uint16_t samples[], uint16_t num_samples, uint16_t sample_rate){
    const uint16_t avgbitwidth = sample_rate / APRS_BAUD;
    uint16_t sampleidx = 0;
    uint16_t lastsampleidx = 0;
    size_t bitbuffidx = 0;
    for(size_t i = 1; i < num_samples; ++i){
        if(samples[i - 1] == 0 && samples[i] == 1){ //transition detected
           if(i - sampleidx > 1){ //ignore false fast tranistions, at least half a bit should've passed
                sampleidx = i + avgbitwidth / 2; //go to the middle of the bit and sample it
                i = sampleidx;
            }
        }
        if(i - sampleidx == avgbitwidth){ //there wasn't a transition, but a bit width has passed
            sampleidx = i;
        }

        if(lastsampleidx != sampleidx){
          //  printf("%d ", sampleidx);
            bitbuff[bitbuffidx++] = (uint8_t)samples[sampleidx];
            lastsampleidx = sampleidx;
        }
    }
    printf("Num of bits found: %d\n", bitbuffidx);
    return bitbuffidx;
}

void reverseNRZI(uint8_t bitbuff[], uint16_t numbits){
    for(uint16_t i = 0; i < numbits - 1; ++i){
        bitbuff[i] = !(bitbuff[i] ^ bitbuff[i+1]);
        //printf("%d %d %d \n", bitbuff[i], bitbuff[i+1], aux);
    }
}

uint8_t getbyte(uint8_t buff[]){
    uint8_t testbyte = 0;
    for(uint8_t i = 0; i < 8; ++i){
        testbyte |= (buff[i] << i);
    }
    return testbyte;
}

uint8_t getbyteinv(uint8_t buff[]){
    uint8_t testbyte = 0;
    for(uint8_t i = 0; i < 8; ++i){
        testbyte |= (buff[7 - i ] << i);
    }
    return testbyte;
}

uint8_t checkbyte(uint8_t byte, uint8_t buff[]){
    return (byte == getbyte(buff));
}

uint16_t getdatastartidx(uint8_t bitbuff[], uint16_t numbits, uint8_t flagcount){
    uint8_t count = 0;
    uint16_t firstflag = (uint16_t)-1;
    uint16_t flagendidx = (uint16_t)-1;
    for(uint16_t i = 0; i < numbits - 8;  ++i){
        if(checkbyte(APRS_FLAG, (&bitbuff[i]))) {
                firstflag = i;
                ++count;
                if(count == flagcount)
                    break;
        }
    }
    if(firstflag != (uint16_t)(-1)){
        for(uint16_t i = firstflag; i < numbits - 8; i += 8){
            if(!checkbyte(APRS_FLAG, (&bitbuff[i]))){ //found the first byte that isn't a flag
                flagendidx = i;
                break;
            }
        }
    }
    return flagendidx;
}

uint16_t getdataendidx(uint8_t bitbuff[], uint16_t numbits, uint16_t datastartidx){
    uint16_t dataendidx = 0;
    for(uint16_t i = datastartidx; i < numbits - 8;  ++i ){
        if(checkbyte(APRS_FLAG, (&bitbuff[i]))) {
                dataendidx = i;
                break;
        }
    }
    return dataendidx;
}

uint16_t removebitstuffing(uint8_t bitbuff[], uint16_t datastartidx, uint16_t dataendidx){
    //i will shift all values to the left if i find a stuffed bit... i think there must be a more efficient solution
    //but i'm too dumb to think of one :)
    uint8_t onecount = 0;
    for(uint16_t i = datastartidx; i < dataendidx; ++i){
        if (bitbuff[i] == 1) ++onecount;
        else onecount = 0;
        if(onecount == 5){
            if(bitbuff[i + 1] == 0) { //found a stuffed bit
                printf("Found a stuffed bit at %d\n", i);
                for(uint16_t j = i + 1; j < dataendidx; ++j){
                    bitbuff[j] = bitbuff[j + 1]; //shift to left
                }
                onecount = 0;
                --dataendidx; //we removed a bit, so decrease the data end
            } else {
                printf("Error - Corrupt data! Bit stuffing not true!\n");
                break;
            }
        }
    } 
    return dataendidx;
}


uint16_t calc_crc (uint8_t bitbuff[], uint16_t datastartidx, uint16_t dataendidx) {
    uint16_t crc = 0xffff; 
    for (uint16_t i = datastartidx; i < dataendidx; i += 8){
        uint8_t byte = getbyte(&bitbuff[i]);
        
        for (uint8_t j = 0; j < 8; j++){
            uint8_t leftbit = (crc & 0x8000) >> 15;
            crc = (crc << 1) & 0xffff;

            if (leftbit ^ (byte & 0x1)) {
                crc = crc ^ 0x1021;
            } 
            byte = byte >> 1;
        }  
    }

    crc = crc ^ 0xffff;

    uint16_t crc_reflect = 0;
    for(uint8_t i = 0; i < 16; ++i){
        crc_reflect |= ((crc & (0x1 << (15 - i))) >> (15 - i)) << i; 
    }

    return crc_reflect;
} 
