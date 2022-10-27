//---------------------------------------------------------------------------

//
//---------------------------------------------------------------------------
#define GPIO_PORTF_LOCK_R       (*((volatile uint32_t *)0x40025520))
#define GPIO_PORTF_PUR_R        (*((volatile uint32_t *)0x40025510))
#define GPIO_PORTF_CR_R         (*((volatile uint32_t *)0x40025524))

// PWM
//#define PWMFREQ 55
#define PWMFREQ 11000
#define SETPOINT 7 // in cm
#define maxVal 9000
#define minVal 1100
#define P_MULT 120
#define I_MULT 0
#define D_MULT 60

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

//extern void IntHandler(void);
//----------------------------------------
// Prototypes
//----------------------------------------
void hardware_init(void);
void ledToggle(void);
void delay(void);
extern void GPIOPadConfigSet(uint32_t ui32Port, uint8_t ui8Pins,
                             uint32_t ui32Strength, uint32_t ui32PadType);
void uart_init(void);
void timer_init(void);
void adc_init(void);
void pwm_init(void);

//UART
void uart_print(const char * , uint32_t);
void uart_cmd_start(void);
void uart_read(void);
void cmd_lookup(char *);

//ADC
float side_sensor_read(void);
float front_sensor_read(void);

// PWM
void pwm_start_motor(void);
void pwm_forward();
void pwm_stop(void);
void pwm_high_speed(void);
void pwm_low_speed(void);
void pwm_set_speed(int16_t, int16_t);

// Float to String
void ftoa(float, char *, int);
void reverse(char *, int);
int intToStr(int, char *, int);

// tasks, interrupts, etc.

void ISR_hwi(void);
void uart_int_handler(void);
void adc_int_handler(void);


//---------------------------------------
// Globals
//---------------------------------------
volatile int16_t i16ToggleCount = 0;
float P , D;
int16_t debugError;
uint16_t leftSpeed = maxVal;
uint16_t rightSpeed = maxVal;
//uint32_t adcVals; // read adc value of side sensor
//uint32_t adcValf; // read adc value of side sensor
int count = 0; // count entered characters in uart
bool uTurn = false;
bool intersection = false;

uint32_t PWMload; // to store load value for PWM
uint16_t duty = 1; // duty 1%


//---------------------------------------------------------------------------
// main()
//---------------------------------------------------------------------------
void main(void)
{
   hardware_init();                         // init hardware via Xware
   float sideDummy = side_sensor_read();
   float frontDummy = front_sensor_read();

   BIOS_start();

}


//---------------------------------------------------------------------------
// hardware_init()
//
// inits GPIO pins for toggling the LED
//---------------------------------------------------------------------------
void hardware_init(void)
{
    //Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    timer_init();
    uart_init();
    adc_init();
    pwm_init();

}

void timer_init(void){
    uint32_t ui32Period;
    // ADD Tiva-C GPIO setup - enables port, sets pins 1-3 (RGB) pins for output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
    // Timer 2 setup code
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);           // enable Timer 2 periph clks
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);        // cfg Timer 2 mode - periodic

    ui32Period = (SysCtlClockGet() /20);                        // period = CPU clk div 2 (500ms)
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period-1);           // set Timer 2 period

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);        // enables Timer 2 to interrupt CPU

    TimerEnable(TIMER2_BASE, TIMER_A);
}

void uart_init(void){
    // enable Port for GPIO and UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Virtual
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Bluetooth
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Virtual
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // bluetooth
    // configure receiver (Rx)
    GPIOPinConfigure(GPIO_PA0_U0RX); // Virtual
    //GPIOPinConfigure(GPIO_PE4_U5RX);
    // configure transmitter (Tx)
    GPIOPinConfigure(GPIO_PA1_U0TX); // Virtual
    //GPIOPinConfigure(GPIO_PE5_U5TX);
    // configure PB0 and PB1 for input
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // configure UART, 9600 8-n-1
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // 115200 for BT, 9600 for virtual
    // enable UART0
    UARTEnable(UART0_BASE);
    // enable interrupts on processor
    IntMasterEnable();
    // enable interrupts on UART0
    IntEnable(INT_UART0);
    // enable interrupts for UART0, Rx and Tx
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void pwm_init(void){
    //SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    //uint32_t PWMclk = SysCtlClockGet() / 64;
    //PWMload = (PWMclk / PWMFREQ) - 1;
    //PWMload = (PWMclk / 100) - 1;
    // set PWM clock

    //Enable Port D for GPIO (use for phase and mode)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //Enable Port A for GPIO (use for motor)
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
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWMload);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWMFREQ);

    // make mode = 1 (high)
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0xFF);

    // set phase
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);

    // Enable Generator 0 with PWM outputs
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMload * duty / 100);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWMload * duty / 100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 0);
    // start motor A and B
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    pwm_start_motor();
}

/// UART ///
// print string to UART
void uart_print(const char * str, uint32_t len){
    uint32_t i;
    for(i = 0; i < len; i++){
        UARTCharPut(UART0_BASE, str[i]);
    }
}

// displays prompt for user to enter command
void uart_cmd_start(void){
    uart_print("Enter Command: ", 15);
}

// read uart input and print output command
void uart_read(void) {
    char cmd_arr[2];
    char cmd_char;
    // read command
    while(UARTCharsAvail(UART0_BASE)){
        // echo input into prompt, non-blocking here
        cmd_char = UARTCharGetNonBlocking(UART0_BASE);
        UARTCharPutNonBlocking(UART0_BASE, cmd_char);
        // store character
        cmd_arr[count] = cmd_char;
        count++;
        // return if we already have two characters
        if(count == 2){
            // carriage return and new line
            UARTCharPut(UART0_BASE, '\r');
            UARTCharPut(UART0_BASE, '\n');
            // reset counter
            count = 0;
            // interpret command
            // print interpretation with uart_print
            cmd_lookup(cmd_arr);
        }
    }
}

void cmd_lookup(char command[]) {
    if (command[0] == 'r' && command[1] == 'e'){
        uart_print ("command ready", 13);
        //pwm_start_motor();
    } else if (command[0] == 'f' && command[1] == 'o'){
        uart_print ("command forward", 15);
        //pwm_forward();
        UARTCharPut(UART0_BASE, '\r');
        UARTCharPut(UART0_BASE, '\n');
        /*while(1){
            front_sensor_read();
            side_sensor_read();
            UARTCharPut(UART0_BASE, '\r');
            UARTCharPut(UART0_BASE, '\n');
        }*/
    } else if (command[0] == 'h' && command[1] == 's'){
        uart_print ("command high speed", 18);
        //pwm_high_speed();
    } else if (command[0] == 'l' && command[1] == 's'){
        uart_print ("command low speed", 17);
        //pwm_low_speed();
    } else if (command[0] == 's' && command[1] == 't'){
        uart_print ("command stop", 12);
        //pwm_stop();
    } else if (command[0] == 's' && command[1] == 'p'){
        uart_print ("command set motor speed", 23);
    } else if (command[0] == 'e' && command[1] == 'r'){
        uart_print ("command error", 13);
    } else {
        uart_print ("command not found", 17);
    }
    UARTCharPut(UART0_BASE, '\r');
    UARTCharPut(UART0_BASE, '\n');
    UARTCharPut(UART0_BASE, '\n');
    uart_cmd_start();
}

void adc_init(void)
{
    // Enable ADC0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // configure PE3 for input
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    // Configure sample sequencer
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_END | ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCSequenceEnable(ADC0_BASE, 2);

}

//---------------------------------------------------------------------------
// ledToggle()
//
// toggles LED on Tiva-C LaunchPad
//---------------------------------------------------------------------------
void ledToggle(void)
{
    // LED values - 2=RED, 4=BLUE, 8=GREEN
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
    }
    Log_info1("LED Toggle %u", i16ToggleCount);
    delay();                                // create a delay of ~1/2sec

    i16ToggleCount += 1;
}

void uart_int_handler(void){
    uart_print("uart_int_handler called", 23);
    UARTCharPut(UART0_BASE, '\r');
    UARTCharPut(UART0_BASE, '\n');
    //TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    uint32_t statusInt;
    // get and clear interrupt status
    statusInt = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, statusInt);
}


int16_t errorPrev = 0;
int16_t totalSummation = 0;
uint16_t PIDval;
void adc_int_handler(void){
    uint32_t sideDist = side_sensor_read();
    uint32_t frontDist = front_sensor_read();
    int16_t errorCurr;
    int16_t diff;


    if(frontDist < 7)
        {
            while(frontDist < 13)
            {
                rightSpeed = maxVal;
                leftSpeed = maxVal;
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0xFF); //reverse direction for left wheel (pin 0)
                pwm_set_speed(leftSpeed,rightSpeed);
                frontDist = front_sensor_read();
            }
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); //reverse direction for left wheel (pin 0)

            pwm_set_speed(leftSpeed/2,rightSpeed/2);
        }
    errorCurr = (SETPOINT - sideDist);

    P = P_MULT * errorCurr; //P should be largest

    //totalSummation += errorCurr;
    //I = I_MULT * totalSummation; //I should be smallest

    diff = errorCurr - errorPrev;

    D = D_MULT * diff; //D should be in the middle

    errorPrev = errorCurr;

    PIDval = (uint16_t)abs(P + D);
    //PIDval is less than 0 when the robot is far from the wall
    //PIDval is greater than 0 when the robot is close to the wall
    debugError = errorCurr;
    if (errorCurr < 0) //far from the wall
    {
        /*
        if (sideDist > 15){
            PIDval = PIDval*2;
        }*/
        rightSpeed = rightSpeed - PIDval; // reduce the right speed
        leftSpeed = maxVal; //maximum -> turn right

    }
    if (errorCurr > 0) //close to the wall
    {
        leftSpeed = leftSpeed - PIDval; //PIDval is positive so reduce the leftspeed
        rightSpeed = maxVal; // maximum -> turn left

    }
        //The robot should run between 10-50 duty cycles
        //setting the maximum and minimum speed
    if(rightSpeed > maxVal)
    {
        rightSpeed = maxVal;
    }
    if(rightSpeed < minVal)
    {
        rightSpeed = minVal;
    }
    if(leftSpeed > maxVal)
    {
        leftSpeed = maxVal;
    }
    if(leftSpeed < minVal)
    {
        leftSpeed = minVal;
    }

    pwm_set_speed(leftSpeed,rightSpeed);



}


//---------------------------------------------------------------------------
// delay()
//
// Creates a 500ms delay via TivaWare fxn
//---------------------------------------------------------------------------
void delay(void)
{
     SysCtlDelay(6700000);      // creates ~500ms delay - TivaWare fxn

}

void ISR_hwi(void) {
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    UARTCharPut(UART0_BASE, '\n');
    uart_print("ISR_hwi called", 14);
    UARTCharPut(UART0_BASE, '\r');
    UARTCharPut(UART0_BASE, '\n');
    uart_int_handler();
    adc_int_handler();
        //Swi_post(swi0);
}

/// Float to String conversion ///

// conversion taken from https://www.geeksforgeeks.org/convert-floating-point-number-string/
// Reverses a string 'str' of length 'len'
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

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

/// ADC ///

float front_sensor_read(void){
    //char cmd_arr[2] = {'s','t'};
    uint32_t adcVals; // read adc value of side sensor
    float distance = 0;
    //char uartOut[8];
    // clear ADC interrupt
    ADCIntClear(ADC0_BASE, 2);
    // trigger ADC sampling
    ADCProcessorTrigger(ADC0_BASE, 2);
    // read voltage
    ADCSequenceDataGet(ADC0_BASE, 2, &adcVals);
    while(!ADCIntStatus(ADC0_BASE, 2, false)){}
    // convert adc value into cm
    distance = 16908*pow(adcVals, -1.01);
    //distance = 22700*pow(adcVals, -1.08476) + 0.8;

    // convert float to string
    //ftoa(distance, uartOut, 4);
    // write output (UART)
    //UARTCharPut(UART0_BASE, '\n');
    //uart_print("Side Distance : ", 17);
    //uart_print(uartOut, 8);

    //if(distance < 4){
        //uart_print("Side Too close", 14);
        //cmd_lookup(cmd_arr);
    //}

    //UARTCharPut(UART0_BASE, '\r');
    //UARTCharPut(UART0_BASE, '\n');
    //delay();
    return distance;
}

float side_sensor_read(void){
    uint32_t adcValf; // read adc value of side sensor
    //char cmd_arr[2] = {'s','t'};
    float distance = 0;
    //char uartOut[8];
    // clear ADC interrupt
    ADCIntClear(ADC0_BASE, 1);
    // trigger ADC sampling
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE,1, false)){}
    // read voltage
    ADCSequenceDataGet(ADC0_BASE, 1, &adcValf);
    // convert adc value into cm
    //distance = 8854.1*pow(adcValf, -0.929);
    distance = 22700*pow(adcValf, -1.08476) + 0.8;
    // convert float to string
    //ftoa(distance, uartOut, 4);
    // write output (UART)
    //UARTCharPut(UART0_BASE, '\n');
    //uart_print("Front Distance: ", 17);
    //uart_print(uartOut, 8);

    /*
    if(distance < 4){
        uart_print("Front Too close", 14+1);
        //cmd_lookup(cmd_arr);
    }*/
    //UARTCharPut(UART0_BASE, '\r');
    //UARTCharPut(UART0_BASE, '\n');
    //delay();
    return distance;
}

/// PWM ///

void pwm_start_motor(void){
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}

void pwm_forward(void){
    //duty = 25;
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMload * duty / 100);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWMload * duty / 100);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, duty);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, duty);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
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

void pwm_set_speed(int16_t left, int16_t right){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, left);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, right);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
}
