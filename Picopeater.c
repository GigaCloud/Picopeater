#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    static bool a;

    gpio_put(LED_PIN, a);

    a = !a;

    add_alarm_in_ms(2000, alarm_callback, NULL, false);

    return 0;
}



int main()
{
    stdio_init_all();

    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);


    while(1) {;}

    puts("Hello, world!");

    return 0;
}
