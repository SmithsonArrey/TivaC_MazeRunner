/*
 * uart_t17_lib.c
 *
 *  Created on: Oct 22, 2022
 *      Author: email
 */
#include "hardware_t17_lib.h"

//globals
int count = 0; // count entered characters in uart

//UART5 on interrupt table is 77
//UART0 on interrupt table is 21

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
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // 115200 for BT, 9600 for virtual
    // enable UART0
    UARTEnable(UART0_BASE);
    // enable interrupts on processor
    IntMasterEnable();
    // enable interrupts on UART0
    IntEnable(INT_UART0);
    // enable interrupts for UART0, Rx and Tx
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
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

void ConfigureUART0(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);



   //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);



   //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);



   //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);



   //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

