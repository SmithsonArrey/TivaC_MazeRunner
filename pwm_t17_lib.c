/*
 * pwm_t17_lib.c
 *
 *  Created on: Oct 22, 2022
 *      Author: email
 */

#include "hardware_t17_lib.h"

// PWM
//#define PWMFREQ 55
#define PWMFREQ 11000
#define SETPOINT 6 // in cm
#define maxVal 9200
#define minVal 900
#define P_MULT 150
#define I_MULT 5
#define D_MULT 75

uint32_t PWMload; // to store load value for PWM
uint16_t duty = 1; // duty 1%

void pwm_init(void){

    //Enable Port D for GPIO (use for phase and mode)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);        // Phase and Mode
    //Enable Port A for GPIO (use for motor)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);        // Motor
    // Set PWM 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    // Set PD0 for phase A
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
    // Set PA6 for PWM output motor A
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PA6_M1PWM2);
    // Set PD1 for phase B
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    // Set PA7 for PWM output motor B
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);
    // Set PD2 for mode
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);

    // Configure Generator 0, counting down
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWMload);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWMFREQ);

    // make mode = 1 (high)
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0xFF);

    // set phase
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);

    // Enable Generator 0 with PWM outputs
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 0);
    // start motor A and B
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
//    pwm_start_motor();
//    pwm_set_speed(5000, 5000);
//    pwm_low_speed();
}


void pwm_start_motor(void){
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}

void pwm_forward(void){
    //duty = 25;
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMload * duty / 100);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWMload * duty / 100);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, duty);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, duty);
    //PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    //PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}

void pwm_reverse(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}


void pwm_right(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
}

void pwm_stop(void){
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
}

void pwm_high_speed(void){
    if (duty < 100){
        duty = duty + 25;
    }
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMload * duty / 100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWMload * duty / 100);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}

void pwm_low_speed(void){
    if (duty > 25){
        duty = duty - 25;
    }
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMload * duty / 100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWMload * duty / 100);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}

void pwm_fast_speed(void){
    pwm_set_speed(15000, 15000);
}

void pwm_slow_speed(void){
    pwm_set_speed(8000, 8000);
}


void pwm_set_speed(int16_t left, int16_t right){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, left);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, right);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}



