/*
 * hardware_t17_lib.h
 *
 *  Created on: Oct 22, 2022
 *      Author: email
 */

#ifndef HARDWARE_T17_LIB_H_
#define HARDWARE_T17_LIB_H_

//------------------------------------------
// TivaWare Header Files
//------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
//#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include <time.h>
#include <xdc/std.h>                        //mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h>                //mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>                //needed for any Log_info() call
#include <xdc/cfg/global.h>                 //header file for statically defined objects/handles

#include <driverlib/pin_map.h>
#include <driverlib/uart.h>
#include "driverlib/adc.h"
#include <math.h>
#include "driverlib/pwm.h"

// FROM SUPPORTING Team 17 Libraries

// FORWARD DECLARATIONS //

//TIMER
void delay(void);
void timer_init(void);

//UART
void uart_init(void);
void uart_print(const char * , uint32_t);
void uart_cmd_start(void);
void uart_read(void);
void cmd_lookup(char *);

//ADC
void adc_init(void);
uint32_t front_read(void);
uint32_t side_read(void);

// PWM
void pwm_init(void);
void pwm_start_motor(void);
void pwm_forward(void);
void pwm_reverse(void);
void pwm_right(void);
void pwm_stop(void);
void pwm_high_speed(void);
void pwm_low_speed(void);
void pwm_set_speed(int16_t, int16_t);

// Float to String
void ftoa(float, char *, int);
void reverse(char *, int);
int intToStr(int, char *, int);

// RGB
void rgb_init(void);
void rgb_set(uint8_t color_index, bool state);
void rgb_toggle(uint8_t color_index);
void ledToggle(void);

enum LED_Colors {
    RED_LED = 0,
    BLUE_LED,
    GREEN_LED};


// INITIALIZATION




#endif /* HARDWARE_T17_LIB_H_ */
