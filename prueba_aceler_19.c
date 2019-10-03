#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include "address_map_arm.h"
#include "interrupt_ID.h"

#define FPGA_tmr_CoReg 1
#define FPGA_tmr_Counter_L 2
#define FPGA_tmr_Counter_H 3

#define JP2_direction   1
#define JP2_imask       2
#define JP2_edge_cpr    3

#define timer_Coreg     2
#define timer_EoIreg    3

volatile int *TR2_ptr;
volatile int *TR3_ptr;
volatile int *JP2_ptr;

volatile int *TR2_ptr;

unsigned short last_trans_pwm_1 = 0;

volatile int *JP1_ptr;
volatile int *FPGA_TR_ptr;
volatile int *LEDR_ptr;

int COUNTER_E0 = 250000; 

unsigned short low2;

unsigned short contador=0;

// Lightweight bridge base address
//  virtual addresses
irq_handler_t irq_handler_fpga_tmr(int irq, void *dev_id, struct pt_regs *regs){

    // After an interrupt occurs, it can be cleared by writing any
    //  value into the Status register.

    if(*(JP1_ptr) & 0b10000){
		low2=1;
	} else{
		low2=0;
	}

    if(low2){
		*(LEDR_ptr)=*(LEDR_ptr) | 0b1;
    } else{
		*(LEDR_ptr)= *(LEDR_ptr) & 0xFFFE;
	}

    *(FPGA_TR_ptr) = *(FPGA_TR_ptr) & 0xFFFE;

    return (irq_handler_t) IRQ_HANDLED;

}

// Lightweight bridge base address
//  virtual addresses
irq_handler_t irq_handler_timer2(int irq, void *dev_id, struct pt_regs *regs){

    *(TR2_ptr + timer_EoIreg);                     // Clear the Edgecapture register (clears current interrupt)

    if (last_trans_pwm_1 == 0){
        *(TR2_ptr + timer_Coreg) = 0b010;          // Stop timer
        *(TR2_ptr) = COUNTER_E0;                   // Load value
        *(JP2_ptr) = 0b0;                          // Set output to 0
        last_trans_pwm_1 = 1;
    } else {
        *(TR2_ptr + timer_Coreg) = 0b010;          // Stop timer
        *(TR2_ptr) = COUNTER_E0;                   // Load value
        *(JP2_ptr) = 0b10000000;                   // Set output to 1
        last_trans_pwm_1 = 0;
    }

    *(TR2_ptr + timer_Coreg) = 0b011;              // Start timers, count from loaded value

    return (irq_handler_t) IRQ_HANDLED;

}

irq_handler_t irq_handler_jp1(int irq, void *dev_id, struct pt_regs *regs){

    // After an interrupt occurs, it can be cleared by writing any
    //  value into the Status register.

    *(FPGA_TR_ptr + FPGA_tmr_CoReg) = 0b0111;                   // Start the counter
    
   	*(LEDR_ptr)=0b10;

    *(JP1_ptr+3) = 0b00;
	low2 = 0;

    return (irq_handler_t) IRQ_HANDLED;

}


static int __init initialize_timer_handler(void){
    void *LW_virtual;
	void * HPS_virtual_timer2;

    int value;

    // generate a virtual address for the FPGA lightweight bridge
    LW_virtual = ioremap_nocache (LW_BRIDGE_BASE, LW_BRIDGE_SPAN);
    JP1_ptr = (unsigned int *)(LW_virtual + JP1_BASE);
    FPGA_TR_ptr = (unsigned int *)(LW_virtual + TIMER_BASE);
    LEDR_ptr = (unsigned int *)(LW_virtual + LEDR_BASE);

    // Timer conf
    *(FPGA_TR_ptr) = 0b00;                                      // Resets the RUN and TO values
    *(FPGA_TR_ptr + FPGA_tmr_CoReg) = 0b1011;                   // Stop the counter
    *(FPGA_TR_ptr + FPGA_tmr_Counter_L) = 0b1001110001000;   	// Low part for 50,000,000
    *(FPGA_TR_ptr + FPGA_tmr_Counter_H) = 0b1;   				// High part for 50,000,000
    *(FPGA_TR_ptr + FPGA_tmr_CoReg) = 0b0111;                   // Start the counter


    *(JP1_ptr) = 0b100000;                        // Clear register, set 
    *(JP1_ptr+1) = 0b0;                             // 0 input 1 output
    *(JP1_ptr+2) = 0b100000;                      // interruption enable --------------------- a lo mejor solo 1

	JP2_ptr = (unsigned int *)(LW_virtual + JP2_BASE);
    
    HPS_virtual_timer2 = ioremap_nocache (HPS_TIMER2, HPS_TIMER2_SPAN);
    TR2_ptr = (unsigned int *)HPS_virtual_timer2;
    
    //Config JP2
    *(JP2_ptr + JP2_direction) = 0b10000000;        // 0 -> Input, 1 -> Output. D7 output
    *(JP2_ptr) = 0b10000000;                        // Clear register, set 

    // TIMERS CONFIG
    // Config TR2
    *(TR2_ptr + timer_Coreg) = 0b010;               // Stop timer
    *(TR2_ptr) = COUNTER_E0;                        // Load value

    // Start timers
    *(TR2_ptr + timer_Coreg) = 0b011;           // Start timers, count from loaded value

    // Interrupt TIMER HPS 2
    value = request_irq (HPS_TIMER2_IRQ, (irq_handler_t) irq_handler_timer2, IRQF_SHARED,
        "timer2_irq_handler", (void *) (irq_handler_timer2));

    // Interrupt JP1
    value = request_irq (JP1_IRQ, (irq_handler_t) irq_handler_jp1, IRQF_SHARED,
        "irq_handler_jp1", (void *) (irq_handler_jp1));

    // Interrupt TIMER HPS 2
    value = request_irq (INTERVAL_TIMER_IRQ, (irq_handler_t) irq_handler_fpga_tmr, IRQF_SHARED,
        "fpga_tmr_irq_handler", (void *) (irq_handler_fpga_tmr));

    return value;

}


static void __exit cleanup_timer_handler(void){

    free_irq (INTERVAL_TIMER_IRQ, (void *) irq_handler_fpga_tmr);
    free_irq (JP1_IRQ, (void *) irq_handler_jp1);
	free_irq (HPS_TIMER2_IRQ, (void *) irq_handler_timer2);
}

module_init(initialize_timer_handler);
module_exit(cleanup_timer_handler);
