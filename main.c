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
int32_t maxSpeed = 2000; //
int32_t minSpeed = 350;
int32_t pwmAdjustLeft, pwmAdjustRight;

int32_t currDiff = 0;

int32_t speed1, speed2;
uint32_t constantADC = 1100; //1350
float P, I, D;
int32_t pulseWidth;


//#define TASKSTACKSIZE   512
//
//Task_Struct task0Struct;
//Char task0Stack[TASKSTACKSIZE];
//extern const ti_sysbios_knl_Semaphore_Handle semPID;
//extern const ti_sysbios_knl_Swi_Handle swi0;

//Menu Functions
void Run();
void PWMDisable();
void LightGet();
void ToggleData();
void PostDriveSemaphore();
void EmergencyStop();
void DriverClockStart();
void stopMotor();

typedef void (*function)();

struct functions {
    char command[2];
    function func;
};

struct functions commands[7] = {
    { "GO", Run },
    { "P0", PWMDisable },
    { "LG", LightGet },
    { "TD", ToggleData },
    { "DS", PostDriveSemaphore },
    { "ES", EmergencyStop },
    { "DC", DriverClockStart }
};

//TIME CALCULATION
int s = 0;
int ms = 0;

//LINE DETECTION
int blackwidth = 0;
int SingleBlackLine = 15;
int line = 0;
void BlackLineDetection();


//DATA COLLECTION
volatile uint16_t prevDiff = 0;
bool recording = false;
//bool running = false;

uint16_t data;

// HELPER FUNCTIONS
void reverse(char* str, int len);
char* itoa(int num, char* str, int base);


/* FUNCTION DEFINITIONS --------------------------------------------------------------------------------------------------------*/
void hardwareInit(void);
void uartInit(void);
void timerInit(void);
void adcInit(void);
void pwmInit(void);
void rgbInit(void);
void gpio_init(void);

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
//Configure 2 timers to get the amount of time it takes robot to complete maze
void MazeTimer(){
    //Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    // WTimer3 Configuration
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER3)) {
    }
    TimerClockSourceSet(WTIMER3_BASE, TIMER_CLOCK_SYSTEM);

    //Configure WTimer3B Periodic Mode
    TimerConfigure(WTIMER3_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR );
    //period = (0.01 sec)
    TimerLoadSet(WTIMER3_BASE, TIMER_B, 40000000/100);
    TimerIntEnable(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);

}
void milliseconds(){
    TimerIntClear(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);

    if(ms == 99){
        ms = 0;
        s++;
    } else {
        ms++;
    }

}


/*This function starts being triggered every 100 ms once the first black
 * single line is cross. The time is enabled in the BlackLineDetection()
 * function. This function handles the data collection. First the buffer
 * is filled. Once full, the buffer is switched and the SWI is called. The
 * SWI is where the green LED is taken care of and the data is transmitter
 * CHAR by CHAR*/

char ping[20];
char pong[20];
int pingCounter = 0;
int pongCounter = 0;
bool pingPrinted = false;
char bufferChar[2];

volatile char *active = ping;

void CollectData(){

    //TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //fill buffer with prevDiff value calculated in PID function

    //format data

    data = (uint16_t)abs(prevDiff);

    if (data == 0){
        bufferChar[0] = '0';
        bufferChar[1] = '0';

    } else {
        itoa(data, bufferChar, 16);
    }

    //fill ping buffer
    if (pongCounter < 20){
        if (pingCounter < 20){
            ping[pingCounter] = bufferChar[0];
            pingCounter++;
            ping[pingCounter] = bufferChar[1];
            pingCounter++;
        } else { // ping is full
            // print ping
            if (!(pingPrinted)){
                Swi_post(swi0);
                pingPrinted = true;
                active = pong;
            }
            // write in pong
            pong[pongCounter] = bufferChar[0];
            pongCounter++;
            pong[pongCounter] = bufferChar[1];
            pongCounter++;
        }
    } else {
    //fill pong buffer
        // pong is full
        // print pong
        Swi_post(swi0);
        pingCounter = 0;
        pongCounter = 0;
        pingPrinted = false;
        active = ping;
    }
}

//SWI transmitting data
void TransmitData(){

    //turn off blue led and turn on green one to indicate data is being transmitted
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);

    UARTCharPut(UART5_BASE, ':');
    UARTCharPut(UART5_BASE, '1');
    UARTCharPut(UART5_BASE, '7');
    int i;

    for(i = 0; i < 20; i++){
      UARTCharPut(UART5_BASE, active[i]);
    }

    UARTCharPut(UART5_BASE, '1');
    UARTCharPut(UART5_BASE, '7');
    UARTCharPut(UART5_BASE, '\r');
    UARTCharPut(UART5_BASE, '\n');

    //turn off green led and turn on blue one to indicate data transmission stopped
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

//Configure a Timer to trigger an interrupt every 5ms
void LightSensorTimerInit(){
    //Configure Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0)) {

    }

    uint32_t Period = SysCtlClockGet();
    //Set Clock Source
    TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM);
    //Configure WTimer2 Periodic Mode
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR );
    //Load WTimer2 with a period of 0.005 s
    TimerLoadSet(WTIMER0_BASE, TIMER_B, (SysCtlClockGet()/1000) * 5);
    //Enable WTimer2 to trigger an interrupt when period is reached
    //IntEnable( INT_TIMER0B );

    TimerIntEnable(WTIMER0_BASE, TIMER_TIMB_TIMEOUT);
    //Enable WTimer2
    TimerEnable(WTIMER0_BASE, TIMER_B);
}

//This function is just the handler for the Light SensorTimer which runs every 5 ms.
//it calls the BlackLineDetection() sensor to handle thin vs thick line and data collection
void LightSensorTimerHandler(){
    TimerIntClear(WTIMER0_BASE, TIMER_TIMB_TIMEOUT);
    BlackLineDetection();
}

void uartInit(void)
{
    // set system clock (will be 40 MHz)
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    // enable Port E for GPIO and UART5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    // configure PE4 as receiver (Rx)
    GPIOPinConfigure(GPIO_PE4_U5RX);
    // configure PE5 as transmitter (Tx)
    GPIOPinConfigure(GPIO_PE5_U5TX);
    // configure PB0 and PB1 for input
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // configure UART, 9600 8-n-1
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    //UARTprintf function

    UARTEnable(UART5_BASE);

    //Enable FIFO
    UARTFIFOEnable(UART5_BASE);
    //UARTFIFOLevelSet(UART5_BASE, UART_FIFO_, UART_FIFO_RX5_8);

    IntMasterEnable();
    IntEnable(INT_UART5);

    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);

    // Initialize the UART for console I/O.
    UARTStdioConfig(1, 115200, SysCtlClockGet());
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

    adcInit();
    pwmInit();
    rgbInit();
    uartInit();
    gpio_init();

    MazeTimer();

    LightSensorTimerInit();

    timerInit();



    startMotor();
    //light sensor is initialized on every call
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

int count = 0;
char command[2];
char commandLet;


void readCommand(void) // UART5 INT HANDLER
{

    UARTIntClear(UART5_BASE, UARTIntStatus(UART5_BASE, true)); //clear the asserted interrupts

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
            UARTCharPut(UART5_BASE, '\r');
            UARTCharPut(UART5_BASE, '\n');
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


void delay_us(uint32_t us) // delay in microseconds
{
     SysCtlDelay(us * 13400);

}


void gpio_init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
}


int read_light_sensor(void){
    int readCycles = 0;
    uint32_t pinVal;
    // handle timers

    // set PB0 high (charge capacitor)
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0);
    SysCtlDelay(200);
    // set PB0 for reading input
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);
    // capture fully charged value
    pinVal = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0);
    // count reading cycles
    while (pinVal & GPIO_PIN_0){
        pinVal = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0);
        readCycles++;
    }
    SysCtlDelay(200);

    return readCycles;

}

//int lineBuffer[2] = {0,0};

int light;
void BlackLineDetection(){


    //Every 5ms this function will read the real-time light sensor value
    light = read_light_sensor();

    /*if the light value is anything above 2000 then it is above a black surface
         * a counter is started to see how many times the BlackLineDetection()
         * function determined the sensor was over a black surface (every 5ms).
         * The blackwidth value will allow us to determine whether the line was thin
         * in thick.
         */
        if(light > 6000)
        {
            blackwidth++;
        }


        /* If light < 5000 then the reflectance sensor is now reading white.
         */
        else {

            /*If blackwidth > 0 then robot just transitioned from black to white. We must
              now determine whether it is a thin or thick line.*/
            if(blackwidth > 0) {

                /*For our robot, if the interrupt has <= 11 black readings
                 * then it is a single black line. We use a global variable counter
                 * to determine if it is the first or second black line.
                 */
                if(blackwidth <= SingleBlackLine){

                    /*if it is the first single black line, then we will turn on blue
                     * LED and start the timer used to start collecting and transmitting data.
                     * The timer enabled triggers the DataTimerHandler where buffer is filled
                     * and SWI is called.*/
                    if(line == 0){
                        //Start Data Collection
                        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
                        recording = true;
                        bluetoothMessage("Recording");
                        //increments the thin line count
                        line++;
                    /*if it is the second single black line, the blue LED is turned off and
                     *the timer is disabled so data collection ends*/
                    } else if(line == 1){
                        //Stop Data Collection
                        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);
                        bluetoothMessage("Not Recording");
                        //Disable 100ms Timer
                        //TimerDisable(WTIMER1_BASE, TIMER_A);
                        recording = false;
                        line = 0;
                    }



                //if time is greater than 11, then it is a thick line and robot is stopped
                } else {
                    EmergencyStop();
                }

                //time is reset to determine thickness of next black line
                blackwidth = 0;

            }
        }

}


uint8_t elapsed_timer_cycles = 0;

void hwi_main_timer(void) // prev ISR_hwi
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    elapsed_timer_cycles++;
    if(elapsed_timer_cycles % 2){
        elapsed_timer_cycles = 0;
        if(recording){
            CollectData();
        }
    }
    else{

    }
    //Semaphore_post(semPID);
    PIDController();
}

void PIDController(void) // task
{

   // while(1){

     //   Semaphore_pend(semPID, BIOS_WAIT_FOREVER);

        int32_t totalSummation = 0;
        int32_t diff;
        frontRead();
        rightRead();
        rgbSwitch(adcValF);

        currDiff = adcValR - constantADC;

        P = P_MULT * currDiff;

        totalSummation += currDiff;
        I = I_MULT * totalSummation;

        diff = currDiff - prevDiff;
        prevDiff = currDiff;
        D = D_MULT * diff;

        pulseWidth = (int32_t) abs(P + I + D);
        Run();

        if (adcValF > 2000)
        {
            while (adcValF > 1500)
            {
                pwmAdjustRight = 2000;
                pwmAdjustLeft = 2000;
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0xFF); //reverse direction for left wheel (pin 0)
                setSpeed(pwmAdjustLeft, pwmAdjustRight);
                frontRead();
            }
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); //reverse direction for left wheel (pin 0)

            setSpeed(pwmAdjustLeft / 2, pwmAdjustRight / 2);
        }
        else
        {
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
            setSpeed(pwmAdjustLeft, pwmAdjustRight);
        }
    //}

}


// int to string
// taken from https://www.geeksforgeeks.org/implement-itoa/
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Implementation of itoa()
char* itoa(int num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;

    /* Handle 0 explicitly, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // In standard itoa(), negative numbers are handled only with
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10)
    {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }

    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';

    str[i] = '\0'; // Append string terminator

    // Reverse the string
    reverse(str, i);

    return str;
}


/***************************MENU FUNCTIONS************************************/
void Run() {
    //Makes the robot start running
    startMotor();
    //forwardMotor();
    TimerEnable(WTIMER3_BASE, TIMER_B); //Enables 1 ms timer
}
void PWMDisable() {
    UARTprintf("PWM Disable\n");
}
void LightGet() {
    UARTprintf("Light Get\n");
}
void ToggleData() {
    UARTprintf("Toggle Data Acquisition\n");
}
void PostDriveSemaphore() {
    UARTprintf("Post Drive Semaphore\n");
}
void EmergencyStop() {
    bluetoothMessage("Emergency Stop\nEmptying Buffer\n");

    stopMotor();
    TimerDisable(WTIMER3_BASE, TIMER_B);
    //UARTprintf("Total Time: %i.%i seconds\n", s, ms);
    bluetoothMessage("Total Time: ");
    char k[2];
    itoa(s,k,10);
    bluetoothMessage(k);
    bluetoothMessage(".");
    itoa(ms,k,10);
    bluetoothMessage(k);
    bluetoothMessage("seconds\n");
    Swi_post(swi0);
    delay();
}
void DriverClockStart() {
    UARTprintf("Driver Clock Start\n");
}



/* MAIN -----------------------------------------------------------------*/
int main(void)
{
    hardwareInit();
    Hwi_enable();
    Swi_enable();
    Task_enable();
    BIOS_start();
}


