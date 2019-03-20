/**
  TMR3 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr3.c

  @Summary
    This is the generated driver implementation file for the TMR3 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides APIs for TMR3.
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

//#include <xc.h>
#include "../main.h"
//#include "tmr3.h"

#define _20us 0x4F  //20 microseconds as interpreted by TMR3 and TMR1
/**
  Section: Global Variables Definitions
*/
volatile uint16_t timer3ReloadVal;
void (*TMR3_InterruptHandler)(void);

/**
  Section: TMR3 APIs
*/

void TMR3_Initialize(void)
{
    //Set the Timer to the options selected in the GUI

    //GSS T3G_pin; TMR3GE disabled; T3GTM disabled; T3GPOL low; T3GGO_nDONE done; T3GSPM disabled; 
    T3GCON = 0x00;

    //TMR 232; 
    TMR3H = 0xE8;

    //TMR 144; 
    TMR3L = 0x90;

    // Load the TMR value to reload variable
    timer3ReloadVal=(uint16_t)((TMR3H << 8) | TMR3L);

    // Clearing IF flag before enabling the interrupt.
    PIR4bits.TMR3IF = 0;

    // Enabling TMR3 interrupt.
    PIE4bits.TMR3IE = 1;

    // Set Default Interrupt Handler
    TMR3_SetInterruptHandler(TMR3_DefaultInterruptHandler);

    // CKPS 1:2; T3OSCEN disabled; nT3SYNC synchronize; CS FOSC/4; TMR3ON enabled; 
    T3CON = 0x11;
}

void TMR3_StartTimer(void)
{
    // Start the Timer by writing to TMRxON bit
    T3CONbits.TMR3ON = 1;
}

void TMR3_StopTimer(void)
{
    // Stop the Timer by writing to TMRxON bit
    T3CONbits.TMR3ON = 0;
}

uint16_t TMR3_ReadTimer(void)
{
    uint16_t readVal;
    uint8_t readValHigh;
    uint8_t readValLow;
    
	
    readValLow = TMR3L;
    readValHigh = TMR3H;
    
    readVal = ((uint16_t)readValHigh << 8) | readValLow;

    return readVal;
}

void TMR3_WriteTimer(uint16_t timerVal)
{
    timerVal += _20us;
    if (T3CONbits.nT3SYNC == 1)
    {
        // Stop the Timer by writing to TMRxON bit
        T3CONbits.TMR3ON = 0;

        // Write to the Timer3 register
        TMR3H = (timerVal >> 8);
        TMR3L = timerVal;

        // Start the Timer after writing to the register
        T3CONbits.TMR3ON =1;
    }
    else
    {
        // Write to the Timer3 register
        TMR3H = (timerVal >> 8);
        TMR3L = timerVal;
    }
}

void TMR3_Reload(void)
{
    TMR3_WriteTimer(timer3ReloadVal);
}

void TMR3_StartSinglePulseAcquisition(void)
{
    T3GCONbits.T3GGO_nDONE = 1;
}

uint8_t TMR3_CheckGateValueStatus(void)
{
    return (T3GCONbits.T3GVAL);
}

void TMR3_ISR(void)
{

    // Clear the TMR3 interrupt flag
    PIR4bits.TMR3IF = 0;
    //TMR3_WriteTimer(timer3ReloadVal); //Always reloaded in TMR3_Callback(), or TMR5_Callback()

    // ticker function call;
    // ticker is 1 -> Callback function gets called everytime this ISR executes
    TMR3_CallBack();
}

void TMR3_CallBack(void)
{
    // Add your custom callback code here
    
    //Save FSRs and others
    uint8_t saveWREG = WREG;
    uint8_t saveBSR = BSR;
    uint8_t saveSTATUS = STATUS;
    uint8_t saveFSR0H = FSR0H;
    uint8_t saveFSR0L = FSR0L;
    uint8_t saveFSR1H = FSR1H;
    uint8_t saveFSR1L = FSR1L;
    
    if(TMR3_InterruptHandler)
    {
        TMR3_InterruptHandler();
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

void TMR3_SetInterruptHandler(void (* InterruptHandler)(void)){
    TMR3_InterruptHandler = InterruptHandler;
}

void TMR3_DefaultInterruptHandler(void){
    // add your TMR3 interrupt custom code
    // or set custom function using TMR3_SetInterruptHandler()
    
    extern struct PORT_Data portData;
    extern struct PWM_Data pwmData;
    
    if(portData.frameEnd == false) {    //if the frame has ended, and a new one has not 
                                // started (20ms have not passed), then exit
        if(portData.iPort < (portData.PORT_SIZE - 1)) {
            LATD = LATD << 1;  //turn previous channel off, and next channel on to create pulse
            portData.iPort++;
            //timer3ReloadVal = pwmData.reg[portData.iPort];
            //TMR3_WriteTimer(timer3ReloadVal);
            TMR3_WriteTimer(pwmData.reg[portData.iPort]);
            
        }
        else {
            LATD = 0;       //clear LATD when all channels have completed their pulse output
            portData.frameEnd = true;   //the frame has ended, and data can be retrieved
        }
    }
}

/**
  End of File
*/
