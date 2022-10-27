/*
 * adc_t17_lib.c
 *
 *  Created on: Oct 22, 2022
 *      Author: email
 */
#include "hardware_t17_lib.h"


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

uint32_t front_read(void){ //ADC0 Seq1
    uint32_t adcVals; // adc values stored in this array

    // clear ADC interrupt
    ADCIntClear(ADC0_BASE, 1);
    // trigger ADC sampling
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE,1, false)){}
    // read voltage
    ADCSequenceDataGet(ADC0_BASE, 1, &adcVals);
    return adcVals; //raw adc value
}
uint32_t side_read(void){ //ADC0 Seq2
    uint32_t adcValf; // adc values stored in this array
    // clear ADC interrupt
    ADCIntClear(ADC0_BASE, 2);
    // trigger ADC sampling
    ADCProcessorTrigger(ADC0_BASE, 2);
    while(!ADCIntStatus(ADC0_BASE,2, false)){}
    // read voltage
    ADCSequenceDataGet(ADC0_BASE, 2, &adcValf);
    return adcValf; //raw adc value
}

uint32_t calc_distance(float sensorVolt){
    uint32_t dist = 14871*pow(sensorVolt, -0.995);
    return dist;
}

