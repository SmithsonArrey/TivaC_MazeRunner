#include "hardware_t17_lib.h"


//extern void IntHandler(void);
//----------------------------------------
// Prototypes

void ISR_hwi(void);
void uart_int_handler(void);
void adc_int_handler(void);
void debug_handler(void);

void hardware_init(void);

void PID_controller(int16_t new_error);

//---------------------------------------
// Globals
//---------------------------------------
// PID
//int PIDval;
//uint16_t PIDval;
//const int maxDuty = 100;
//float err;
//float prevErr;
//float deriv;
//float integr;
//float targetDist = 7; // ... cm from wall
//float frontTargDist = 4;
//float openSide = 10;

//  //

//
#define PWMFREQ 11000
#define SETPOINT 2474 //rep in mV //6 cm
#define maxVal 9200
#define minVal 900
#define kp 150
#define ki 5
#define kd 75

int16_t old_error = 0;
int16_t total_summation = 0;

int pause_counter = 0;
//
//---------------------------------------------------------------------------
// main()
//---------------------------------------------------------------------------
void main(void)
{
   hardware_init();                         // init hardware via Xware
   BIOS_start();
}

//                        //

void hardware_init(void)
{
    //Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    timer_init();
    uart_init();
    adc_init();
    pwm_init();

}


// HARDWARE INTERRUPT FUNCTIONS //

void ISR_hwi(void) { // rename this to TIMER2A_INT_HANDLER
    if(pause_counter >= 20){
        Swi_post(swi1);
    }
    UARTCharPut(UART0_BASE, '\n');
    uart_print("ISR_hwi called", 14);
    UARTCharPut(UART0_BASE, '\r');
    UARTCharPut(UART0_BASE, '\n');
    uart_int_handler(); //
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    Swi_post(swi0);
    pause_counter++;
}

///////////////////////////////////

// SOFTWARE INTERRUPT FUNCTIONS //


void adc_int_handler(void){ // rename this to SENSOR_RESPONSE_PROCESSING or something

    PID_controller(SETPOINT - front_read());//side_read());


    //calc_distance(front_read());

}

void debug_handler(void){

    pwm_stop();

    delay(); // allows program to pause to check debug messages
    delay();

    pwm_start_motor();
    pause_counter = 0;
}



//////////////////////////////////



/// HELPER FUNCTIONS ///

//controller will maintain a distance of 6 cm in front of the robot
void PID_controller(int16_t new_error){// 1D example. 2D would require 2D new_error and such
    // Calculate P
    int16_t P = (kp * new_error)/1000;
    // Calculate I
    int16_t I = total_summation + ((ki * (new_error + old_error))/1000);
    // Calculate D
    int16_t D = (kd * (new_error - old_error))/1000;

    //set direction
    if ((P+I+D) > 0){ //far from wall
        pwm_forward();
    }
    else if ((P+I+D) == 0){
        pwm_stop();
    }
    else{//too close to wall
        pwm_reverse();
    }
    //set speed
    int16_t SPEED = abs(P + I + D); //do I need to convert this to something a pwm would take
    pwm_set_speed(SPEED, SPEED);

    // update old_error
    old_error = new_error;

    total_summation = I;

}

void uart_int_handler(void){ //rename this to uart_debug_update or something , not an actual interrupt
    uart_print("uart_int_handler called", 23);
    UARTCharPut(UART0_BASE, '\r');
    UARTCharPut(UART0_BASE, '\n');
    //TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    uint32_t statusInt;
    // get and clear interrupt status
    statusInt = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, statusInt);
}

