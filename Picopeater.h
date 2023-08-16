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

#define ADC_SAMPLING_RATE 44100 //Hz

#define ABS(X) (X >= 0? X : -X)



typedef union{
    struct{
        uint8_t signalDetected : 1;
        uint8_t signalEnded : 1;
    };
    uint16_t bytes;
} interruptFlags_t;