#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "debug.h"
#include "Picopeater.h"

#include "dsp/filtering_functions.h"

#include "ax25.h"

#define SAMPLE_BUFFER_SIZE 39600
#define BIT_BUFFER_SIZE 1200
int16_t buff[SAMPLE_BUFFER_SIZE];
uint8_t bitbuff[BIT_BUFFER_SIZE];
#define MIN_SAMPLES 10000 //about 250ms @ 39600Hz

volatile uint16_t val;
volatile uint8_t draBuff[100];
volatile uint16_t adcSample;
uint16_t neutralAdcValue; 

volatile uint16_t sampleNumber;
volatile uint8_t overflowCounter;

interruptFlags_t interruptFlags;

bool processedSignal = true;

static inline void ledToggle(){
    gpio_put(LED_PIN, !gpio_get_out_level(LED_PIN));
}


void adcCallback(void){
    adcSample = adc_fifo_get();
    adc_fifo_drain();

    int16_t diff = adcSample - neutralAdcValue;

    if(processedSignal) {
        buff[sampleNumber++] = diff;
     
        if(sampleNumber == SAMPLE_BUFFER_SIZE) {
            sampleNumber = 0;
            ++overflowCounter;
        }
    }
}


void on_uart_rx(){
    static uint8_t rxIdx;
    while (uart_is_readable(UART_DRA_ID)) {
        draBuff[rxIdx++] = uart_getc(UART_DRA_ID);
        if(rxIdx == DRA_BUFF_SIZE)
            rxIdx = 0;
    }
}


void squelchCallback(uint8_t gpio, uint32_t events){
    if(GPIO_IRQ_EDGE_FALL == events) { //detected a signal
        interruptFlags.signalDetected = 1;
        if(processedSignal){
            sampleNumber = 0;
            gpio_put(LED_PIN, true);
            adc_run(true);
        }
    }

    if(GPIO_IRQ_EDGE_RISE == events){ //signal ended
        interruptFlags.signalEnded = 1;
        adc_run(false);
        gpio_put(LED_PIN, false);
        processedSignal = false;
    }
}


void init(){
    stdio_init_all();
    
    gpio_init(LED_PIN);

    gpio_init(DRA_PD_PIN);
    gpio_init(DRA_SQ_PIN);

    gpio_set_dir(DRA_PD_PIN, GPIO_OUT);
    gpio_set_dir(DRA_SQ_PIN, GPIO_IN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_put(DRA_PD_PIN, 1);

    uart_init(UART_DRA_ID, 9600);

    gpio_set_function(UART_DRA_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_DRA_RX_PIN, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_DRA_ID, false, false);
    uart_set_format(UART_DRA_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_DRA_ID, false);

    adc_init();
    adc_gpio_init(ADC_AF_OUT_PIN);
    adc_select_input(ADC_CH_AF_OUT);

    uint32_t adcClockDiv = CLOCK_FREQ / ADC_SAMPLING_RATE;
    adc_set_clkdiv(adcClockDiv);
    adc_fifo_setup(true, false, 1, false, false);

    irq_set_exclusive_handler(ADC_IRQ_FIFO, (irq_handler_t)adcCallback);
    
    irq_set_exclusive_handler(UART_DRA_IRQ, (irq_handler_t)on_uart_rx);
    irq_set_enabled(UART_DRA_IRQ, true);
    
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_DRA_ID, true, false);

    

    sleep_ms(500);
}



void printAPRS(uint8_t bitbuff[], int datastartidx, int dataendidx){
    printf("APRS DATA, datastartidx %d: \n", datastartidx);
    int addressendidx = -1;
    for(int i = datastartidx; i < dataendidx; i += 8){
        uint8_t byte = getbyte(&bitbuff[i]);
        bool lastaddressbyte = byte & 0x1;
        byte = byte >> 1;
        printf("%c", (char)byte);
        if(lastaddressbyte) {
            addressendidx = i + 8;
            break;
        }
    } 

    int aprsdataidx = 0;
    for(int i = addressendidx; i < dataendidx; i+=8){
        uint8_t byte = getbyte(&bitbuff[i]);
        if(byte == 0x03){
            byte = getbyte(&bitbuff[i + 8]);
            if (byte == 0xF0){
                aprsdataidx = i + 16;                
                break;
            } else {
                printf("Error: Control field and protocol id found! \n");
                break;
            }
        }

        printf("%c", (char)byte); //print digipeater address if present
    }

    for(int i = aprsdataidx;  i < dataendidx; i += 8){
        uint8_t byte = getbyte(&bitbuff[i]);
        printf("%c", (char)byte);
    }    
}


void reverse(int *array, int start, int end) {
    while (start < end) {
        int temp = array[start];
        array[start] = array[end];
        array[end] = temp;
        start++;
        end--;
    }
}

void rearrangeArray(int *array, int sampleIdx, int buffsize) {
    // Determine the number of elements to rotate
    int rotateIdx = (sampleIdx) % buffsize;
    
    // If no rotation is needed, return
    if (rotateIdx == 0) {
        return;
    }
    
    // Reverse the entire array
    reverse(array, 0, buffsize - 1);
    
    // Reverse the first part of the array (from 0 to buffsize - rotateIdx - 1)
    reverse(array, 0, buffsize - rotateIdx - 1);
    
    // Reverse the second part of the array (from buffsize - rotateIdx to buffsize - 1)
    reverse(array, buffsize - rotateIdx, buffsize - 1);
}



int main()
{
    init();
    printf("Picopeter 0.1\n");
    //uart_puts(UART_DRA_ID, "AT+DMOCONNECT\r\n");
    // Timer example code - This example fires off the callback after 2000ms

    neutralAdcValue = adc_read();

    //AT+DMOSETGROUP=GBW,TFV, RFV,Tx_CTCSS,SQ,Rx_CTCSS<
    uart_puts(UART_DRA_ID, "AT+DMOSETGROUP=0,144.8000,144.8000,0000,1,0000\r\n");
    sleep_ms(100);
    uart_puts(UART_DRA_ID, "AT+DMOSETVOLUME=8\r\n");


    sleep_ms(100);

    adc_irq_set_enabled(true);
    irq_set_enabled(ADC_IRQ_FIFO, true);

   // adc_run(true);

    gpio_set_irq_enabled_with_callback( DRA_SQ_PIN, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        (gpio_irq_callback_t) squelchCallback
    );


    while(1) {


        if(interruptFlags.signalDetected){
              interruptFlags.signalDetected = 0;
                //printf("Signal detected!\n");
            
        }

        if(interruptFlags.signalEnded){
            interruptFlags.signalEnded = 0;
            //printf("Signal ended!\n");
        }


        if(!processedSignal){
            if(sampleNumber > MIN_SAMPLES || overflowCounter > 0){
                printf("Processing the signal, sampleidx at %d with overflowcounter at %d...\n", sampleNumber, overflowCounter);

                if(overflowCounter > 0) {
                    printf("Buffer overflowed, restoring index...");
                    rearrangeArray(buff, sampleNumber, SAMPLE_BUFFER_SIZE);
                    sampleNumber = SAMPLE_BUFFER_SIZE;
                }

                for(uint16_t i = 0; i < sampleNumber; ++i){
                    printf("%d\n", buff[i]);
                } 

                runCorrelator(buff, sampleNumber, SAMPLE_RATE);

                uint16_t numbits = getBits(bitbuff, buff, sampleNumber, SAMPLE_RATE);
                reverseNRZI(bitbuff, numbits);

                int datastartidx = getdatastartidx(bitbuff, numbits, 5);
                printf("Data start idx: %d\n", datastartidx);
                int dataendidx = getdataendidx(bitbuff, numbits, datastartidx);
                printf("Initial dataendidx: %d\n", dataendidx);
                dataendidx = removebitstuffing(bitbuff, datastartidx, dataendidx);
                printf("Dataendidx after removing stuffed bits: %d\n", dataendidx);

                printf("Found data start at: %d; Found data end at: %d\n", datastartidx, dataendidx);

                printf("\n");

                uint16_t crc = calc_crc(bitbuff, datastartidx, dataendidx - 16);

                uint16_t actualcrc = getbyte(&bitbuff[dataendidx - 8]) << 8 | getbyte(&bitbuff[dataendidx - 16]);

                if(crc == actualcrc) {
                    printf("CRC matched! \n");
                } else {
                    printf("Error! CRC do not match, calc: %x, actual %x\n", crc, actualcrc);
                }

                printAPRS(bitbuff, datastartidx, dataendidx);

                /*
                printf("\nPrinting raw packet bytes: \n");
                
                for(int i = datastartidx; i < dataendidx; i+=8){
                    printf("%x ", getbyte(&bitbuff[i]));
                } */

                printf("\nData ended\n", getbyte(&bitbuff[dataendidx + 1]));
            }
            processedSignal = true;
            sampleNumber = 0;
            overflowCounter = 0;
        }
    }

    return 0;
}
