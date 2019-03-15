/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

//#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

#include "mcc_generated_files/mcc.h"
#include <stdbool.h>

// TODO Insert declarations

// Definitions
#define _1MS_COMP       0xE0C0  //1ms as interpreted by timer1 and timer3
#define _1_5MS_COMP     0xD120  //1.5ms as interpreted by timer1 and timer3
#define _2MS_COMP       0xC180  //2ms as interpreted by timer1 and timer3
#define _RESOLUTION     _1MS-_2MS
#define _4MS_COMP       0x8300
#define _6MS_COMP       0x4480  //For Detecting Break Pulse

// PPM Definitions
#define _PPM_BUF_SIZE           9  // 6 channels + 3 modes (manual drive and drive to manipulation switch)
#define _I_PPM_BUF_MANUAL_MODE  0   //Index to dataBuf
#define _I_PPM_BUF_CTRL_MODE    1
#define _I_PPM_BUF_AUTO_MODE    2
#define _I_PPM_BUF_DATA_START   3   //Number of modes above

#define _PPM_REG_SIZE           6   // 6 channels
#define _I_PPM_REG_DATA_START   0

//#define _STATE_PPM_READY            0
//#define _STATE_PPM_BREAK_RECEIVED   1

// UART Definitions
//#define _STATE_UART_READY       0
//#define _STATE_G_RECEIVED       1
//#define _STATE_O_RECEIVED       2
//#define _STATE_PID_GO_RECEIVED  3
//#define _STATE_A_RECEIVED       4
//#define _STATE_T_RECEIVED       5
//#define _STATE_PID_AT_RECEIVED  6

// UART_GO Definitions   (for receiving drive commands)
#define _UART_BUF_GO_SIZE       8   //6 speed bytes + 1 direction byte + 1 byte for CRC
#define _I_UART_GO_DIR          6   //Index for direction byte
#define _I_UART_GO_CRC          7
#define _I_UART_GO_DATA_START   0

// UART_AT definitions   (for receiving End Point Initialization)
#define _UART_BUF_AT_SIZE       13  // [6 channels]*2 (2 byte values for each channel) + 1 byte CRC
#define _I_UART_AT_CRC          12
#define _I_UART_AT_DATA_START   0


// TODO Insert C++ class definitions if appropriate

//Struct Definitions
struct PPM_Data {
    const size_t BUF_SIZE = _PPM_BUF_SIZE;
    const size_t I_MANUAL_MODE = _I_PPM_BUF_MANUAL_MODE;
    const size_t I_CTRL_MODE = _I_PPM_BUF_CTRL_MODE;
    const size_t I_AUTO_MODE = _I_PPM_BUF_AUTO_MODE;
    const size_t I_BUF_DATA_START = _I_PPM_BUF_DATA_START;
    
    const size_t REG_SIZE = _PPM_REG_SIZE;
    const size_t I_REG_DATA_START = _I_PPM_REG_DATA_START;
    
    const enum LoadState{READY, BREAK_RECEIVED};
    
    uint16_t buf[BUF_SIZE];     //contains timer values as received
    size_t iBuf;
    uint16_t reg[REG_SIZE];     //contains filtered data
    size_t iReg;
    size_t ep_reg[2*REG_SIZE];    //end point register set (used for filtering buf data before entering reg)
    
    PPM_Data();
    //size_t state;         //tracks the state of the PPM input (0: waiting for break pulse, 1: break pulse received)
    void PPMRead();
    void EndFrame();    //Ends and validates PPM frame at period clock interrupt
    bool ppmValid();
    void UpdateReg();
    bool IsManualMode();    //returns true if in manual control mode (overrides autonomous control)
    bool IsManipulationMode();     //returns true if in manipulation mode
    bool IsAutoMode();      //returns true if autonomy is enabled AND manual mode is disabled
}ppmData;

struct UART_Data {
    //GO command constants
    const size_t BUF_GO_SIZE = _UART_BUF_GO_SIZE;
    const size_t I_DIR = _I_UART_GO_DIR;
    const size_t I_CRC = _I_UART_GO_CRC;
    const size_t I_GO_DATA_START = _I_UART_GO_DATA_START;
    
    //AT initialization command constants
    const size_t BUF_AT_SIZE = _UART_BUF_AT_SIZE;
    const size_t I_AT_DATA_START = _I_UART_AT_DATA_START;
    
    //RAM allocation for GO data
    uint8_t buf[BUF_AT_SIZE];     //contains byte data values sent via UART (used for GO and AT packets, notice that AT packets are larger)
    size_t iBuf;
    
    //RAM allocation for AT data
    uint8_t ep_reg[BUF_AT_SIZE];
    
    enum LoadState{READY, G_RECEIVED, O_RECEIVED, A_RECEIVED, 
                    T_RECEIVED, PID_GO_RECEIVED, PID_AT_RECEIVED};
    
    UART_Data();
    bool CheckCRC();
    void UpdateBuf(PPM_Data* &data);    //Translates data, and sends it to PPM_Data->buf;
    bool IsGOPacketReady();   //Returns true if whole packet has been received
    bool IsATPacketReady();
    void LoadByte();        //loads byte from Receive Register
    //state made into an enum: uint8_t loadState;     //holds the current state of data packet reception
    bool goPacketReady;
    bool atPacketReady;
}uartData;

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */



