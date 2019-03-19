/**
  CCP1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    ccp1.c

  @Summary
    This is the generated driver implementation file for the CCP1 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides implementations for driver APIs for CCP1.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.76
        Device            :  PIC16F1777
        Driver Version    :  2.1.3
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

#include <xc.h>
#include "ccp1.h"


volatile CCP1_PERIOD_REG_T lastCCP1Val;

/**
  Section: Capture Module APIs:
*/


void CCP1_Initialize(void)
{
    // Set the CCP1 to the options selected in the User Interface
	
	// MODE Rising edge; EN enabled; FMT right_aligned; 
	CCP1CON = 0x85;    
	
	// RH 0; 
	CCPR1H = 0x00;    
	
	// RL 0; 
	CCPR1L = 0x00;    
	
	// CTS CCP1 pin; 
	CCP1CAP = 0x00;    
    
    // Last CCP 1 value starts at timer high time
    lastCCP1Val.ccpr1_16Bit = 0xFFFF;

    
}

bool CCP1_IsCapturedDataReady(void)
{
    // Check if data is ready to read from capture module by reading "CCPIF" flag.
    bool status = PIR1bits.CCP1IF;
    if(status)
    PIR1bits.CCP1IF = 0;
    return (status);
}

uint16_t CCP1_CaptureRead(void)
{
    CCP1_PERIOD_REG_T module;
    CCP1_PERIOD_REG_T temp;
    
    // Copy captured value.
    temp.ccpr1l = CCPR1L;
    temp.ccpr1h = CCPR1H;
    
    //subtract last value read
    module.ccpr1_16Bit = temp.ccpr1_16Bit - lastCCP1Val.ccpr1_16Bit;
    
    //update last CCP1 value
    lastCCP1Val.ccpr1_16Bit = temp.ccpr1_16Bit;
    
    // Return 16bit captured value
    return module.ccpr1_16Bit;
}
/**
 End of File
*/