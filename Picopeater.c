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

#define true 1
#define false 0 

volatile uint16_t val;
volatile uint8_t draBuff[100];
volatile uint16_t adcSample;
uint16_t neutralAdcValue; 

interruptFlags_t interruptFlags;

static inline void ledToggle(){
    gpio_put(LED_PIN, !gpio_get_out_level(LED_PIN));
}


void adcCallback(void){
    //ledToggle();

    adcSample = adc_fifo_get();
    adc_fifo_drain();

    int16_t diff = adcSample - neutralAdcValue;

    if(ABS(diff) > 100) {
        gpio_put(LED_PIN, true);
    }
    else {
        gpio_put(LED_PIN, false);
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
    if(GPIO_IRQ_EDGE_FALL == events) {
        interruptFlags.signalDetected = 1;
        adc_run(true);
    }

    if(GPIO_IRQ_EDGE_RISE == events){
        interruptFlags.signalEnded = 1;
        adc_run(false);
        gpio_put(LED_PIN, false);
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

int main()
{
    init();
    printf("Picoprobe 0.1\n");
    //uart_puts(UART_DRA_ID, "AT+DMOCONNECT\r\n");
    // Timer example code - This example fires off the callback after 2000ms

    neutralAdcValue = adc_read();

    uart_puts(UART_DRA_ID, "AT+DMOSETGROUP=0,144. 8000,144.8000,0000,1,0000\r\n");
    sleep_ms(100);
    uart_puts(UART_DRA_ID, "AT+DMOSETVOLUME=8\r\n");


    sleep_ms(100);

    adc_irq_set_enabled(true);
    irq_set_enabled(ADC_IRQ_FIFO, true);

   // adc_run(true);

    gpio_set_irq_enabled_with_callback( DRA_SQ_PIN, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        squelchCallback
    );


    

    while(1) {

#if DEBUG_ON
        if(interruptFlags.signalDetected){
              interruptFlags.signalDetected = 0;
            printf("Signal detected!\n");
        }

        if(interruptFlags.signalEnded){
            interruptFlags.signalEnded = 0;
            printf("Signal ended!\n");
        }
#endif
    }

    return 0;
}
