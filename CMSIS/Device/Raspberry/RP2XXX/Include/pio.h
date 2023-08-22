#ifndef __PICO_PIO_H
#define __PICO_PIO_H

#ifdef __cplusplus
extern "C"
{
#endif

#define SWDIO_PIN 4
#define SWCLK_PIN 5

#define MSB_FIRST 0
#define LSB_FIRST 1

    typedef struct _pio_sm_exec_config {
        uint32_t STATUS_N : 4;      // Comparison level for the MOV x, STATUS instruction
        uint32_t STATUS_SEL : 1;    // Comparison used for the MOV x, STATUS instruction.
        uint32_t resv : 2;
        uint32_t WRAP_BOTTOM : 5;      // After reaching wrap_top, execution is wrapped to this address.
        uint32_t WRAP_TOP : 5;         // After reaching this address, execution is wrapped to wrap_bottom.
        uint32_t OUT_STICKY : 1;       // Continuously assert the most recent OUT/SET to the pins
        uint32_t INLINE_OUT_EN : 1;    // If 1, use a bit of OUT data as an auxiliary write enable When used in conjunction with OUT_STICKY.
        uint32_t OUT_EN_SEL : 5;       // Which data bit to use for inline OUT enable
        uint32_t JMP_PIN : 5;          // The GPIO number to use as condition for JMP PIN. Unaffected by input mapping.
        uint32_t SIDE_PINDIR : 1;      // If 1, side-set data is asserted to pin directions, instead of pin values
        uint32_t SIDE_EN : 1;          // If 1, the MSB of the Delay/Side-set instruction field is used as side-set enable, rather than a side-set data bit.
        uint32_t EXEC_STALLED : 1;     // If 1, an instruction written to SMx_INSTR is stalled, and latched by the state machine.
    } pio_sm_exec_config;

    typedef struct _pio_sm_shift_config {
        uint32_t : 16;
        uint32_t AUTOPUSH : 1;        // Push automatically when the input shift register is filled
        uint32_t AUTOPULL : 1;        // Pull automatically when the output shift register is emptied
        uint32_t IN_SHIFTDIR : 1;     // 1 = shift input shift register to right (data enters from left). 0 = to left.
        uint32_t OUT_SHIFTDIR : 1;    // 1 = shift out of output shift register to right. 0 = to left.
        uint32_t PUSH_THRESH : 5;     // Number of bits shifted into ISR before autopush, or conditional push (PUSH IFFULL), will take place. Write 0 for value of 32.
        uint32_t PULL_THRESH : 5;     // Number of bits shifted out of OSR before autopull, or conditional pull (PULL IFEMPTY), will take place. Write 0 for value of 32.
        uint32_t FJOIN_TX : 1;        // When 1, TX FIFO steals the RX FIFO’s storage, and becomes twice as deep.
        uint32_t FJOIN_RX : 1;        // When 1, RX FIFO steals the TX FIFO’s storage, and becomes twice as deep.
    } pio_sm_shift_config;

    typedef struct _pio_sm_pinctrl_config {
        uint32_t OUT_BASE : 5;         // The lowest-numbered pin that will be affected by an OUT PINS
        uint32_t SET_BASE : 5;         // The lowest-numbered pin that will be affected by a SET PINS or SET PINDIRS instruction.
        uint32_t SIDESET_BASE : 5;     // The lowest-numbered pin that will be affected by a side- set operation.
        uint32_t IN_BASE : 5;          // The pin which is mapped to the least-significant bit of a state machine’s IN data bus.
        uint32_t OUT_COUNT : 6;        // The number of pins asserted by an OUT PINS, OUT PINDIRS or MOV PINS instruction.
        uint32_t SET_COUNT : 3;        // The number of pins asserted by a SET. In the range 0 to 5 inclusive.
        uint32_t SIDESET_COUNT : 3;    // The number of MSBs of the Delay/Side-set instruction
    } pio_sm_pinctrl_config;

    typedef struct _pio_sm_registers {
        volatile uint32_t CLKDIV;
        volatile uint32_t EXECCTRL;
        volatile uint32_t SHIFTCTRL;
        volatile uint32_t ADDR;
        volatile uint32_t INSTR;
        volatile uint32_t PINCTRL;
    } pio_sm_registers;

    extern volatile pio_sm_registers *PIO0_SM;
    extern volatile pio_sm_registers *PIO1_SM;

#define PIO_SM_0 (1 << 0)
#define PIO_SM_1 (1 << 1)
#define PIO_SM_2 (1 << 2)
#define PIO_SM_3 (1 << 3)

#define PIO_SET_BASE_SET(PIOx, sm, base, count) (MODIFY_REG(##PIOx##_SM[sm].PINCTRL, PIO0_SM0_PINCTRL_SET_BASE_Msk | PIO0_SM0_PINCTRL_SET_COUNT_Msk, \
                                                            (base) << PIO0_SM0_PINCTRL_SET_BASE_Pos | (count) << PIO0_SM0_PINCTRL_SET_COUNT_Pos))

#define PIO_SET_BASE_SIDESET(PIOx, sm, base, count) (MODIFY_REG(##PIOx##_SM[sm].PINCTRL, PIO0_SM0_PINCTRL_SIDESET_BASE_Msk | PIO0_SM0_PINCTRL_SIDESET_COUNT_Msk, \
                                                                (base) << PIO0_SM0_PINCTRL_SIDESET_BASE_Pos | (count) << PIO0_SM0_PINCTRL_SIDESET_COUNT_Pos))

#define PIO_SET_BASE_OUT(PIOx, sm, base, count) (MODIFY_REG(##PIOx##_SM[sm].PINCTRL, PIO0_SM0_PINCTRL_OUT_BASE_Msk | PIO0_SM0_PINCTRL_OUT_COUNT_Msk, \
                                                            (base) << PIO0_SM0_PINCTRL_OUT_BASE_Pos | (count) << PIO0_SM0_PINCTRL_OUT_COUNT_Pos))

#define PIO_SET_BASE_IN(PIOx, sm, base) (MODIFY_REG(##PIOx##_SM[sm].PINCTRL, PIO0_SM0_PINCTRL_IN_BASE_Msk, \
                                                    (base) << PIO0_SM0_PINCTRL_IN_BASE_Pos))

#define PIO_SET_PC(PIOx, sm, pc) (PIOx##_SM[sm].INSTR = pio_encode_jmp(pc))

#define PIO_RX_FIFO(PIOx, sm) (((volatile uint32_t *) &PIOx->RXF0)[sm])
#define PIO_TX_FIFO(PIOx, sm) (((volatile uint32_t *) &PIOx->TXF0)[sm])

#define PIO_GET_TX_FIFO_LEVEL(PIOx, sm) (PIOx->FLEVEL >> (sm * 0x08) & 0xF)
#define PIO_GET_RX_FIFO_LEVEL(PIOx, sm) (PIOx->FLEVEL >> (sm * 0x08 + 0x04) & 0xF)

    /**
     * @brief Get current SM configuration
     *
     * @param PIOx
     * @param sm state machine number (0 - 3)
     * @param div requested clock_divider value
     * @param exec_config pointer to a pio_sm_exec_config structure
     * @param shift_config pointere to a pio_sm_shift_config structure
     */
    void pio_get_config_sm(PIO0_Type *PIOx, uint8_t sm, float *div, pio_sm_exec_config *exec_config, pio_sm_shift_config *shift_config);

    /**
     * @brief Set new SM configuration
     *
     * @param PIOx
     * @param sm state machine number (0 - 3)
     * @param div requested clock_divider value
     * @param exec_config pointer to a pio_sm_exec_config structure
     * @param shift_config pointere to a pio_sm_shift_config structure
     */
    void pio_set_config_sm(PIO0_Type *PIOx, uint8_t sm, float div, pio_sm_exec_config *exec_config, pio_sm_shift_config *shift_config);

    /**
     * @brief Set new pinctrl configuration
     *
     * @param PIOx
     * @param sm state machine number (0 - 3)
     * @param pinctrl_config pointer to a pio_sm_pinctrl_config structure
     */
    void pio_set_pinctrl_sm(PIO0_Type *PIOx, uint8_t sm, pio_sm_pinctrl_config *pinctrl_config);

    /**
     * @brief Enable State Machine
     *
     * @param PIOx
     * @param sm state machine bitmask (for example to enable sm 1 us 0x1)
     */
    void pio_enable(PIO0_Type *PIOx, uint8_t sm);

    /**
     * @brief Disable State Machine
     *
     * @param PIOx
     * @param sm state machine bitmask (for example to enable sm 1 us 0x1)
     */
    void pio_disable(PIO0_Type *PIOx, uint8_t sm);

    /**
     * @brief Execute instruction on specific state machine
     *
     * @param PIOx
     * @param sm state machine number (0 - 3)
     * @param instr 16bit PIO instruction
     */
    void pio_execute_instr(PIO0_Type *PIOx, uint8_t sm, uint16_t instr);

    /**
     * @brief
     *
     * @param PIOx
     * @param buffer
     * @param size
     */
    void pio_program(PIO0_Type *PIOx, const uint16_t *buffer, uint16_t size);

    /**
     * @brief 
     * 
     * @param PIOx 
     * @param sm 
     * @param peri_clk 
     * @param requested_clk 
     * @return uint32_t 
     */
    uint32_t pio_set_clock(PIO0_Type *PIOx, uint8_t sm, uint32_t peri_clk, uint32_t requested_clk);

    /**
     * @brief
     *
     * @param PIOx
     */
    void pio_init(PIO0_Type *PIOx);

    /**
     * @brief
     *
     * @param PIOx
     */
    void pio_deinit(PIO0_Type *PIOx);

    /**
     * @brief
     *
     * @param PIOx
     * @param sm
     */
    void pio_drain_fifo(PIO0_Type *PIOx, uint8_t sm);

#ifdef __cplusplus
}
#endif

#endif