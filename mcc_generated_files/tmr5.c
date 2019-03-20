/**
  TMR5 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr5.c

  @Summary
    This is the generated driver implementation file for the TMR5 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides APIs for TMR5.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.76
        Device            :  PIC16F1777
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.00
        MPLAB 	          :  MPLAB X 5.10
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/**
  Section: Included Files
*/

/**
 * Timer 5 is the PWM period timer (20 ms timer)
 */

//#include <xc.h>
//#include "../main.h"
//#include "tmr5.h"
//#include "tmr3.h"
#include "../main.h"

/**
  Section: Global Variables Definitions
*/
volatile uint16_t timer5ReloadVal;
void (*TMR5_InterruptHandler)(void);

/**
  Section: TMR5 APIs
*/

void TMR5_Initialize(void)
{
    //Set the Timer to the options selected in the GUI

    //GSS T5G_pin; TMR5GE disabled; T5GTM disabled; T5GPOL low; T5GGO_nDONE done; T5GSPM disabled; 
    T5GCON = 0x00;

    //TMR 99; 
    TMR5H = 0x63;

    //TMR 192; 
    TMR5L = 0xC0;

    // Load the TMR value to reload variable
    timer5ReloadVal=(uint16_t)((TMR5H << 8) | TMR5L);

    // Clearing IF flag before enabling the interrupt.
    PIR4bits.TMR5IF = 0;

    // Enabling TMR5 interrupt.
    PIE4bits.TMR5IE = 1;

    // Set Default Interrupt Handler
    TMR5_SetInterruptHandler(TMR5_DefaultInterruptHandler);

    // CKPS 1:4; T5OSCEN disabled; nT5SYNC synchronize; CS FOSC/4; TMR5ON enabled; 
    T5CON = 0x21;
}

void TMR5_StartTimer(void)
{
    // Start the Timer by writing to TMRxON bit
    T5CONbits.TMR5ON = 1;
}

void TMR5_StopTimer(void)
{
    // Stop the Timer by writing to TMRxON bit
    T5CONbits.TMR5ON = 0;
}

uint16_t TMR5_ReadTimer(void)
{
    uint16_t readVal;
    uint8_t readValHigh;
    uint8_t readValLow;
    
	
    readValLow = TMR5L;
    readValHigh = TMR5H;
    
    readVal = ((uint16_t)readValHigh << 8) | readValLow;

    return readVal;
}

void TMR5_WriteTimer(uint16_t timerVal)
{
    if (T5CONbits.nT5SYNC == 1)
    {
        // Stop the Timer by writing to TMRxON bit
        T5CONbits.TMR5ON = 0;

        // Write to the Timer5 register
        TMR5H = (timerVal >> 8);
        TMR5L = timerVal;

        // Start the Timer after writing to the register
        T5CONbits.TMR5ON =1;
    }
    else
    {
        // Write to the Timer5 register
        TMR5H = (timerVal >> 8);
        TMR5L = timerVal;
    }
}

void TMR5_Reload(void)
{
    TMR5_WriteTimer(timer5ReloadVal);
}

void TMR5_StartSinglePulseAcquisition(void)
{
    T5GCONbits.T5GGO_nDONE = 1;
}

uint8_t TMR5_CheckGateValueStatus(void)
{
    return (T5GCONbits.T5GVAL);
}

void TMR5_ISR(void)
{

    // Clear the TMR5 interrupt flag
    PIR4bits.TMR5IF = 0;
    TMR5_WriteTimer(timer5ReloadVal);

    // ticker function call;
    // ticker is 1 -> Callback function gets called everytime this ISR executes
    
    TMR5_CallBack();
}

void TMR5_CallBack(void)
{
    // Add your custom callback code here
    //struct PORT_Data portData = GetPortData();
    //struct PWM_Data pwmData = GetPwmData();
    
    //Save FSRs and others
    uint8_t saveWREG = WREG;
    uint8_t saveBSR = BSR;
    uint8_t saveSTATUS = STATUS;
    uint8_t saveFSR0H = FSR0H;
    uint8_t saveFSR0L = FSR0L;
    uint8_t saveFSR1H = FSR1H;
    uint8_t saveFSR1L = FSR1L;
    
    
    if(TMR5_InterruptHandler)
    {
        TMR5_InterruptHandler();
    }
    
    //Return FSRs and others
    FSR0H = saveFSR0H;
    FSR0L = saveFSR0L;
    FSR1H = saveFSR1H;
    FSR1L = saveFSR1L;
    STATUS = saveSTATUS;
    BSR = saveBSR;
    WREG = saveWREG;
}

void TMR5_SetInterruptHandler(void (* InterruptHandler)(void)){
    TMR5_InterruptHandler = InterruptHandler;
}

void TMR5_DefaultInterruptHandler(void){
    // add your TMR5 interrupt custom code
    // or set custom function using TMR5_SetInterruptHandler()
    
    extern struct PORT_Data portData;
    extern struct PWM_Data pwmData;
    
    portData.frameEnd = false;  //is set true when the last expected PPM pulse 
                                //  is received. Performed by TMR3_Callback()
    portData.iPort = 0;         //start counting PPM pulses from 0
    LATD = 0x01;                //First channel is at PORTD 0
    //timer3ReloadVal = pwmData.reg[portData.iPort];
    //TMR3_Reload(timer3ReloadVal);   //!!!Make sure this is all I need to do to set TMR3
    TMR3_WriteTimer(pwmData.reg[portData.iPort]);   //!!!Make sure this is all I need to do to set TMR3
    
    driveWDT++; //increment drive watch dog timer every 20ms
    
}

/**
  End of File
*/
