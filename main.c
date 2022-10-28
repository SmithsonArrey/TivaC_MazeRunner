/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* HEADERS -------------------------------------------------------------*/
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Log.h>
#include <xdc/cfg/global.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "utils/uartstdio.h"

/* DEFINITIONS -------------------------------------------------------*/
#define maxVal 9000 // max speed
#define minVal 900 // min speed
#define PWMFREQ 2600
#define SETPOINT 6 // in cm
//#define P_MULT 150
//#define I_MULT 5
//#define D_MULT 75

/* GLOBAL VARIABLES ---------------------------------------------------*/
uint32_t adcValF; // front reading
uint32_t adcValR; // right reading
int16_t leftSpeed = maxVal; // left motor
int16_t rightSpeed = maxVal; // right motor
uint32_t ui32Period;
volatile uint32_t interruptCount;
uint32_t PWMload; // to store load value for PWM
uint16_t duty = 1; // duty 1%
const int maxDuty = 100;
float voltageRead;
int16_t errorPrev = 0;
int16_t totalSummation = 0;
float P_MULT = 0.7;
float I_MULT = 0.01;
float D_MULT = 0.0015;
int32_t maxSpeed = 1000; // PWM max = 1000
int32_t minSpeed = 350;
int32_t pwmAdjustLeft, pwmAdjustRight;
int32_t currDiff = 0;
int32_t prevDiff = 0;
int32_t speed1, speed2;
uint32_t constantADC = 1000; //1350
float P, I, D;
int32_t pulseWidth;
Swi_Handle swi0, swi1;

/* FUNCTION DEFINITIONS --------------------------------------------------------------------------------------------------------*/
void hardwareInit(void);
void uartInit(void);
void timerInit(void);
void adcInit(void);
void pwmInit(void);
void rgbInit(void);

void bluetoothMessage(char *array);

void rightRead(void);
void frontRead(void);
int32_t check_speed(int32_t speed);

void startMotor(void);
void stopMotor(void);
void forwardMotor(void);
void setSpeed(int16_t, int16_t);

void delay(void);

void PIDController(void);

/* INITIALIZATION FUNCTIONS ----------------------------------------------------------------------------------------------------*/
void uartInit(void)
{
    // bluetooth
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // configure UART, 9600 8-n-1
    UARTConfigSetExpClk(
            UART5_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // 115200 for BT, 9600 for virtual
    // enable UART0
    UARTEnable(UART5_BASE);
    // enable interrupts on processor
    IntMasterEnable();
    // enable interrupts on UART0
    IntEnable(INT_UART5);
    // enable interrupts for UART0, Rx and Tx
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
}

void adcInit(void)
{
// Enable ADC0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
// configure PE3 for input
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);
// Configure sample sequencer
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0,
    ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0,
    ADC_CTL_CH1 | ADC_CTL_END | ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCSequenceEnable(ADC0_BASE, 2);
}

void rgbInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
    GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
// start with turning on green light
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 4);
}

void pwmInit(void)
{
    //motor ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
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
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
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
}

void timerInit(void)
{
    uint32_t ui32Period = 40000 - 1;
    // ADD Tiva-C GPIO setup - enables port, sets pins 1-3 (RGB) pins for output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
    GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 4);
    // Timer 2 setup code
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);  // enable Timer 2 periph clks
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // cfg Timer 2 mode - periodic

    // period = CPU clk div 2 (500ms)
    TimerPrescaleSet(TIMER2_BASE, TIMER_A, 50 - 1);
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);        // set Timer 2 period

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 2 to interrupt CPU

    TimerEnable(TIMER2_BASE, TIMER_A);
}

void hardwareInit(void)
{
//Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
    SysCtlClockSet(
    SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    timerInit();
//    uartInit();
    adcInit();
    pwmInit();
    rgbInit();
    startMotor();
    setSpeed(8000, 8000);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // for light sensor
}

/* BLUETOOTH PRINT ------------------------------------------*/
void bluetoothMessage(char *array)
{
    while (*array)
    {
        UARTCharPut(UART5_BASE, *array);
        array++;
    }
}

void commandInterpreter(char command[])
{
    const char *keys[5];
    int leds[5];

    keys[0] = "GG"; //start motor
    keys[1] = "SS"; //stop motor
    keys[2] = "LL"; //low speed
    keys[3] = "MM"; //medium speed
    keys[4] = "HH"; //high speed

    leds[0] = 8; //start green
    leds[1] = 2; //stop red
    leds[2] = 4; //change speed blue
    leds[3] = 5; //change speed blue
    leds[4] = 6; //change speed blue

    int keyvalue = -1; /* I assume -1 is not in the value array, so it can be an error condition that we didn't find that key when searching in the first array */
    int i = 0;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    for (i; i < 5; i++)
    {
        if (strcmp(command, keys[i]) == 0)
        {
            keyvalue = leds[i];
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                         keyvalue);
            switch (keyvalue)
            {
            case 2:
                bluetoothMessage("\nStop motor\n");
                stopMotor();
                break;
            case 4:
                bluetoothMessage("\nLow speed\n");
                setSpeed(1000, 1000);
                break;
            case 5:
                bluetoothMessage("\nMedium speed\n");
                setSpeed(4000, 4000);
                break;
            case 6:
                bluetoothMessage("\nHigh speed\n");
                setSpeed(8000, 8000);
                break;
            case 8:
                bluetoothMessage("\nStart motor\n");
                startMotor();
                break;
//            case 0:
//                bluetoothMessage("\nLed off\n");
//                break;
            default:
                bluetoothMessage("\nInvalid command\n");
            }
            break;
        }
    }

    SysCtlDelay(10000000);
}

void readCommand(void)
{
    int count = 1;
    char command[2];
    char commandLet;

    while (UARTCharsAvail(UART5_BASE))
    {
        commandLet = UARTCharGetNonBlocking(UART5_BASE);
        UARTCharPutNonBlocking(UART5_BASE, commandLet);
// store character
        command[count] = commandLet;
        count++;
// return if we already have two characters
        if (count == 2)
        {
            // carriage return and new line
            UARTCharPut(UART0_BASE, '\r');
            UARTCharPut(UART0_BASE, '\n');
            // reset counter
            count = 0;
            // interpret command
            // print interpretation with uart_print
            commandInterpreter(command);
        }
    }
}

/* SENSOR READING --------------------------------------------------------*/
void frontRead(void)
{
// clear ADC interrupt
    ADCIntClear(ADC0_BASE, 1);
// trigger ADC sampling
    ADCProcessorTrigger(ADC0_BASE, 1);
    while (!ADCIntStatus(ADC0_BASE, 1, false))
    {
    }
// read voltage
    ADCSequenceDataGet(ADC0_BASE, 1, &adcValF);
}

void rightRead(void)
{
// clear ADC interrupt
    ADCIntClear(ADC0_BASE, 2);
// trigger ADC sampling
    ADCProcessorTrigger(ADC0_BASE, 2);
    while (!ADCIntStatus(ADC0_BASE, 2, false))
    {
    }
// read voltage
    ADCSequenceDataGet(ADC0_BASE, 2, &adcValR);
}

int32_t check_speed(int32_t speed)
{
    if (speed > maxSpeed)
    {
        speed = maxSpeed;
    }
    else if (speed < minSpeed)
    {
        speed = minSpeed;
    }
    return speed;
}

/* LED -----------------------------------------------------------------*/
void rgbSwitch(uint32_t sensorVolt)
{
    if (sensorVolt > 3000)
    { // 4 cm critical - red
        GPIOPinWrite(GPIO_PORTF_BASE,
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                     2);
    }
    else if (sensorVolt > 1000)
    { // 7 cm warning - yellow
        GPIOPinWrite(GPIO_PORTF_BASE,
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                     10);
    }
    else
    { // further than 7 cm away, green
        GPIOPinWrite(GPIO_PORTF_BASE,
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                     8);
    }
}

/* OTHER FUNCTIONS -----------------------------------------------------------------*/
void delay(void)
{
    SysCtlDelay(6700000); // ~500ms delay
}

/* MOTOR ----------------------------------------------------------------*/
void startMotor(void)
{
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}

void stopMotor(void)
{
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
}

void forwardMotor(void)
{
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2);
}

void setSpeed(int16_t left, int16_t right)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, left);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, right);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}

void uart_int_handler(void)
{
    uint32_t statusInt;
    statusInt = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, statusInt);
}

void ISR_hwi(void)
{
    uart_int_handler();
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    Swi_post(swi0);
}

void PIDController(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    int32_t totalSummation = 0;
    int32_t diff;
    rightRead(); //stored in adcValR
    frontRead(); //stored in adcValF
    rgbSwitch(adcValR);

    currDiff = adcValR - constantADC;

    P = P_MULT * currDiff;

    totalSummation += currDiff;
    I = I_MULT * totalSummation;

    diff = currDiff - prevDiff;
    prevDiff = currDiff;
    D = D_MULT * diff;

    pulseWidth = (int32_t) abs(P + I + D);

//    if (adcValF < 1000)
//    {
//        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0xFF); // 1 : backward
//        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00); // 0: forward
//        pwmAdjustLeft = maxSpeed;
//        pwmAdjustRight = maxSpeed;
//    }

    startMotor();
    if (currDiff < -1)
    {
        speed1 = pwmAdjustLeft - pulseWidth;
        speed2 = pwmAdjustRight + pulseWidth;
        pwmAdjustLeft = check_speed(speed1);
        pwmAdjustRight = check_speed(speed2);
    }
    else if (currDiff > 1)
    {
        speed1 = pwmAdjustLeft + pulseWidth;
        speed2 = pwmAdjustRight - pulseWidth;
        pwmAdjustLeft = check_speed(speed1);
        pwmAdjustRight = check_speed(speed2);
    }
    else
    {
        pwmAdjustLeft = 8000;
        pwmAdjustRight = 8000;
    }

    setSpeed(pwmAdjustLeft, pwmAdjustRight); // purposefully flipping it because our wiring is opposite
}

/* MAIN -----------------------------------------------------------------*/
int main(void)
{
    hardwareInit();
//    startMotor();
    PIDController();
    BIOS_start();
}

