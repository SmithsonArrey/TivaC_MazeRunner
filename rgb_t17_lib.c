/*
 * rgb_t17_lib.c
 *
 *  Created on: Oct 22, 2022
 *      Author: email
 */

#include "hardware_t17_lib.h"


void rgb_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //front sensor PE3
    //side(right) sensor PE2
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    // start with turning on green light
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
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
    //Log_info1("LED Toggle %u", i16ToggleCount);
    delay();                                // create a delay of ~1/2sec

}


void rgb_set(uint8_t color_index, bool state){
    //changes the state of one of the onboard LEDs

    if(color_index == RED_LED){
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, state);

    }
    else if(color_index == BLUE_LED){
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, state);

        }
    else if(color_index == GREEN_LED){
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, state);
        }
}
void rgb_toggle(uint8_t color_index){
    //toggles one of the onboard LEDs

    int32_t pin_states = (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2) & 0xF);


    if(color_index == RED_LED){
        if(pin_states > 0){
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
        }
        else{
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 1);
        }
    }
    else if(color_index == BLUE_LED){
        if(pin_states > 0){
           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
           }
        else{
           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 1);
        }
    }
    else if(color_index == GREEN_LED){
        if(pin_states > 0){
           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
        }
        else{
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 1);
            }
       }
}

void rgb_distance_switch(int distance)
{
    if (distance < 6){ // 6 cm critical - red
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
        //UARTprintf("red - critical");
        //UARTprintf("\n");
    } else if (distance < 10){ // 10 cm warning - yellow
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 10);
    //UARTprintf("yellow - warning");
    //UARTprintf("\n");
    } else { // furtheer than 10 cm away, green
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
    //UARTprintf("green");
    //UARTprintf("\n");

    }
    //delay();
}
