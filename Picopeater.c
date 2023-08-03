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


const uint LED_PIN = PICO_DEFAULT_LED_PIN;



#define ADC_CH_AF_OUT 0
#define DRA_PD_PIN 3

#define UART_DRA_ID uart0
#define UART_DRA_IRQ UART0_IRQ
#define UART_DRA_TX_PIN 0
#define UART_DRA_RX_PIN 1

#define DRA_BUFF_SIZE 100

#define ADC_AF_OUT_PIN 26

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    static bool a;

    gpio_put(LED_PIN, a);

    a = !a;

    add_alarm_in_ms(2000, alarm_callback, NULL, false);

    return 0;
}




volatile uint16_t val;

volatile uint8_t draBuff[100];

void on_uart_rx(){
    static uint8_t rxIdx;
    while (uart_is_readable(UART_DRA_ID)) {
        draBuff[rxIdx++] = uart_getc(UART_DRA_ID);
        if(rxIdx == DRA_BUFF_SIZE)
            rxIdx = 0;
    }
}

void init(){
    stdio_init_all();


    
    gpio_init(LED_PIN);
    gpio_init(ADC_AF_OUT_PIN);
    gpio_init(DRA_PD_PIN);
    gpio_set_dir(DRA_PD_PIN, GPIO_OUT);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_set_dir(ADC_AF_OUT_PIN, GPIO_IN);
    gpio_put(DRA_PD_PIN, 1);

    uart_init(UART_DRA_ID, 9600);

    gpio_set_function(UART_DRA_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_DRA_RX_PIN, GPIO_FUNC_UART);


    adc_init();
    adc_select_input(ADC_CH_AF_OUT);


    uart_set_hw_flow(UART_DRA_ID, false, false);
    uart_set_format(UART_DRA_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_DRA_ID, false);
    
    irq_set_exclusive_handler(UART_DRA_IRQ, on_uart_rx);
    irq_set_enabled(UART_DRA_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_DRA_ID, true, false);
    sleep_ms(500);
}

int main()
{
    init();

    //uart_puts(UART_DRA_ID, "AT+DMOCONNECT\r\n");
    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);


    uart_puts(UART_DRA_ID, "AT+DMOSETGROUP=0,144.8000,144.8000,0000,1,0000\r\n");
    sleep_ms(100);
    uart_puts(UART_DRA_ID, "AT+DMOSETVOLUME=8\r\n");


    while(1) {
       val = adc_read();
    }

    return 0;
}
