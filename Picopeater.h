#define LED_PIN PICO_DEFAULT_LED_PIN

#define CLOCK_FREQ 48000000 //Hz

#define DRA_PD_PIN 3
#define DRA_SQ_PIN 4

#define UART_DRA_ID uart0
#define UART_DRA_IRQ UART0_IRQ
#define UART_DRA_TX_PIN 0
#define UART_DRA_RX_PIN 1

#define DRA_BUFF_SIZE 100

#define ADC_CH_AF_OUT 0
#define ADC_AF_OUT_PIN (26 + ADC_CH_AF_OUT) //GPIO 26 - CH 0

#define ADC_SAMPLING_RATE 39600//Hz

#define MARK_SIZE 33
#define SPACE_SIZE 18
#define SAMPLE_RATE ADC_SAMPLING_RATE

#define APRS_BAUD 1200
#define MARK_FREQ 1200
#define SPACE_FREQ 2200

#define APRS_FLAG 0b01111110

#define MARK_SIZE 33
#define SPACE_SIZE 18


#define ABS(X) (X >= 0? X : -X)

#define true 1
#define false 0 


typedef union{
    struct{
        uint8_t signalDetected : 1;
        uint8_t signalEnded : 1;
    };
    uint16_t bytes;
} interruptFlags_t;