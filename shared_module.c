#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include "address_map_arm.h"
#include "interrupt_ID.h"

#define JP_direction        1
#define JP_imask            2
#define JP_edge_cpr         3

#define FPGA_tmr_CoReg      1
#define FPGA_tmr_Counter_L  2
#define FPGA_tmr_Counter_H  3

#define timer_Coreg         2
#define timer_EoIreg        3

unsigned int TEMP_COUNTERX_E0 = 467500;
unsigned int TEMP_COUNTERX_E1 = 32500;
unsigned int TEMP_COUNTERY_E0 = 467500; 
unsigned int TEMP_COUNTERY_E1 = 32500;

unsigned int COUNTERX_E0;
unsigned int COUNTERX_E1;
unsigned int COUNTERY_E0;
unsigned int COUNTERY_E1;
		
volatile int *TR2_ptr;
volatile int *TR3_ptr;
volatile int *JP2_ptr;
volatile int *JP1_ptr;
volatile int *FPGA_TR_ptr;
volatile int *LEDR_ptr;

unsigned int counterx, countery;
unsigned short acx_level, acy_level;

unsigned short last_trans_pwm_1 = 0;
unsigned short last_trans_pwm_2 = 0;

unsigned short moduloServoOn=0;
unsigned short moduloAcelerometroOn=0;

unsigned short recalculateX = 0;
unsigned short recalculateY = 0;

unsigned short irqResult1, irqResult2;


// Interrupción Timer HPS 2 -> PWM 1 -> X
irq_handler_t irq_handler_timer2(int irq, void *dev_id, struct pt_regs *regs){

    *(TR2_ptr + timer_EoIreg);                      // Clear the Edgecapture register (clears current interrupt)

    if (last_trans_pwm_1 == 0){
        *(TR2_ptr + timer_Coreg) = 0b010;           // Stop timer
        *(TR2_ptr) = COUNTERX_E0;                   // Load value
        *(JP2_ptr) = *(JP2_ptr) & 0xFFFFFF7F;       // Clear register (bit 7), set output to 0
		
		COUNTERX_E0 = TEMP_COUNTERX_E0;				//Refresh PWM
		COUNTERX_E1 = TEMP_COUNTERX_E1;
		
        last_trans_pwm_1 = 1;
    } else {
        *(TR2_ptr + timer_Coreg) = 0b010;           // Stop timer
        *(TR2_ptr) = COUNTERX_E1;                    // Load value
        *(JP2_ptr) = *(JP2_ptr) | 0x80;             // Write register (bit 7), set output to 1
        last_trans_pwm_1 = 0;
    }

    *(TR2_ptr + timer_Coreg) = 0b011;               // Start timers, count from loaded value

    return (irq_handler_t) IRQ_HANDLED;

}


// Interrupción Timer HPS 3 -> PWM 2 -> Y
irq_handler_t irq_handler_timer3(int irq, void *dev_id, struct pt_regs *regs){

    *(TR3_ptr + timer_EoIreg);                      // Clear the Edgecapture register (clears current interrupt)

    if (last_trans_pwm_2 == 0){
        *(TR3_ptr + timer_Coreg) = 0b010;           // Stop timer
        *(TR3_ptr) = COUNTERY_E0;                   // Load value
        *(JP2_ptr) = *(JP2_ptr) & 0xFFFFFDFF;       // Clear register (bit 9), set output to 0
		
		COUNTERY_E0 = TEMP_COUNTERY_E0;				//Refresh PWM
		COUNTERY_E1 = TEMP_COUNTERY_E1;
		
        last_trans_pwm_2 = 1;
    } else {
        *(TR3_ptr + timer_Coreg) = 0b010;           // Stop timer
        *(TR3_ptr) = COUNTERY_E1;                   // Load value
        *(JP2_ptr) = *(JP2_ptr) | 0x200;            // Write register (bit 9), set output to 1
        last_trans_pwm_2 = 0;
    }

    *(TR3_ptr + timer_Coreg) = 0b011;               // Start timers, count from loaded value

    return (irq_handler_t) IRQ_HANDLED;

}


// Interrupción FPGA Interval Timer -> Sampler
irq_handler_t irq_handler_fpga_tmr(int irq, void *dev_id, struct pt_regs *regs){

    // After an interrupt occurs, it can be cleared by writing any
    //  value into the Status register.
	int grados;
	
	acx_level=(*JP1_ptr);					
	
    if(acx_level & 0b10000000){			// bit 7 eje X
		counterx++;
		recalculateX = 1;
    } else if (recalculateX){
		grados = counterx*23-1150;
		if(grados<-20) grados=-20;
		else if(grados>20) grados=20;
		
		grados= 15;
		
		COUNTERX_E1 = 37500+25*grados;
		COUNTERX_E0 = 500000 - COUNTERX_E1;
		counterx = 0;
	    recalculateX = 0;
	}
	
    if(acy_level & 0b00100000){			// bit 5 eje Y
        countery++;
		recalculateY = 1;
    } else if (recalculateY){
		grados = counterx*23-1150;
		if(grados<-20) grados=-20;
		else if(grados>20) grados=20;
		TEMP_COUNTERY_E1 = 37500+25*grados;
		TEMP_COUNTERY_E0 = 500000 - TEMP_COUNTERX_E1;
		countery = 0;
	    recalculateY = 0;
	}


    *(FPGA_TR_ptr) = 0b00;

    return (irq_handler_t) IRQ_HANDLED;

}



uint16_t EstadoAcelerometro(void){
	return moduloAcelerometroOn;
}



uint16_t InicializarModuloAcelerometro(void * LW_virtual){

    // First initializations
    counterx=0;
    countery=0;

	TEMP_COUNTERX_E0 = 462500;    // timeout = counter/25MHz = 18500us
	TEMP_COUNTERX_E1 = 37500;     // timeout = counter/25MHz =  1500us
	TEMP_COUNTERY_E0 = 462500;    // timeout = counter/25MHz = 18500us
	TEMP_COUNTERY_E1 = 37500;     // timeout = counter/25MHz =  1500us
	
	JP1_ptr = (unsigned int *)(LW_virtual + JP1_BASE);
	*(JP1_ptr+1) = 0b0;                             // 0 input 1 output
	
    LEDR_ptr = (unsigned int *)(LW_virtual + LEDR_BASE);
	
    FPGA_TR_ptr = (unsigned int *)(LW_virtual + TIMER_BASE);    // Timer for signal sampling
	
	// TIMERS CONFIG
    // FPGA Interval Timer
    *(FPGA_TR_ptr) = 0b00;                                      // Resets the RUN and TO values
    *(FPGA_TR_ptr + FPGA_tmr_CoReg) = 0b1011;                   // Stop the counter
    *(FPGA_TR_ptr + FPGA_tmr_Counter_L) = 0b1001110001000;      // Low part for 5000  || 0.1 ms
    *(FPGA_TR_ptr + FPGA_tmr_Counter_H) = 0b0;                  // High part for 5000 || 0.1 ms
    *(FPGA_TR_ptr + FPGA_tmr_CoReg) = 0b0111;                   // Start the counter
	
	// Interrupt Interval Timer
    irqResult2 = request_irq (INTERVAL_TIMER_IRQ, (irq_handler_t) irq_handler_fpga_tmr, IRQF_SHARED,
        "fpga_tmr_irq_handler", (void *) (irq_handler_fpga_tmr));
		
	moduloAcelerometroOn=irqResult2;
	return irqResult2;
}


uint16_t EstadoModuloServos(void){
	return moduloServoOn;
}



uint16_t InicializarModuloServos(void * LW_virtual){

    void * HPS_virtual_timer2;
    void * HPS_virtual_timer3;
	
    JP2_ptr = (unsigned int *)(LW_virtual + JP2_BASE);          // Write PWMs
	
    // Map HPS timer for PWM1 --> HPS TIMER 2
    HPS_virtual_timer2 = ioremap_nocache (HPS_TIMER2, HPS_TIMER2_SPAN);
    TR2_ptr = (unsigned int *)HPS_virtual_timer2;

    // Map HPS timer for PWM2 --> HPS TIMER 3
    HPS_virtual_timer3 = ioremap_nocache (HPS_TIMER3, HPS_TIMER3_SPAN);
    TR3_ptr = (unsigned int *)HPS_virtual_timer3;
    
    //Config JP2
    *(JP2_ptr + JP_direction) = 0b1010000000;                   // 0 -> Input, 1 -> Output. D7 output
    *(JP2_ptr) = 0b10000000;                                    // Clear register, set 


    // Config TR2 -> Servo X
    *(TR2_ptr + timer_Coreg) = 0b010;                           // Stop timer
    *(TR2_ptr) = COUNTERX_E1;                                   // Load value

    // Config TR3 -> Servo Y
    *(TR3_ptr + timer_Coreg) = 0b010;                           // Stop timer
    *(TR3_ptr) = COUNTERY_E1;                                   // Load value

    // Start timers
    *(TR2_ptr + timer_Coreg) = 0b011;                           // Start timers, count from loaded value
    *(TR3_ptr + timer_Coreg) = 0b011;                           // Start timers, count from loaded value
	
	    // Interrupt TIMER HPS 2
    irqResult1 = request_irq (HPS_TIMER2_IRQ, (irq_handler_t) irq_handler_timer2, IRQF_SHARED,
        "timer2_irq_handler", (void *) (irq_handler_timer2));

    // Interrupt TIMER HPS 3
    irqResult2 = request_irq (HPS_TIMER3_IRQ, (irq_handler_t) irq_handler_timer3, IRQF_SHARED,
        "timer3_irq_handler", (void *) (irq_handler_timer3));
		
	moduloServoOn=irqResult2;
	
	return irqResult1&irqResult2;
}



static int __init initialize_proyecto_handler(void){

    void * LW_virtual;

    // generate a virtual address for the FPGA lightweight bridge
    LW_virtual = ioremap_nocache (LW_BRIDGE_BASE, LW_BRIDGE_SPAN);

	InicializarModuloAcelerometro(LW_virtual);
	InicializarModuloServos(LW_virtual);

    return 0;
}


static void __exit cleanup_proyecto_handler(void){

    free_irq (HPS_TIMER2_IRQ, (void *) irq_handler_timer2);
    free_irq (HPS_TIMER3_IRQ, (void *) irq_handler_timer3);
    free_irq (INTERVAL_TIMER_IRQ, (void *) irq_handler_fpga_tmr);

}

module_init(initialize_proyecto_handler);
module_exit(cleanup_proyecto_handler);
