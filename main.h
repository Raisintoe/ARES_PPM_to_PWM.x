/* 
 * File:   main.h
 * Author: derek
 *
 * Created on March 13, 2019, 10:34 PM
 */

//#include <stdbool.h>
#include "mcc_generated_files/mcc.h"

//!!! driver Specific Definitions !!! must change for programming controllers
#define _PIC_IS_DRIVE_CONT      true

// Definitions
#define _DIR_FORWARD    1       //UART direction interpretation
#define _DIR_REVERSE    0

#define _1MS_COMP       0xE0C0  //1ms as interpreted by timer1 and timer3
#define _1_5MS_COMP     0xD120  //1.5ms as interpreted by timer1 and timer3
#define _2MS_COMP       0xC180  //2ms as interpreted by timer1 and timer3
#define _RESOLUTION     _1MS-_2MS
#define _4MS_COMP       0x8300
#define _6MS_COMP       0x4480  //For Detecting Break Pulse

#define _1MS            0xFFFF - _1MS_COMP
#define _1_5MS          0xFFFF - _1_5MS_COMP
#define _2MS            0xFFFF - _2MS_COMP
#define _4MS            0xFFFF - _4MS_COMP
#define _6MS            0xFFFF - _6MS_COMP

//GO UART command definitions 
#define _PID_DRIVE      0x00    //UART Packet ID for drive

//AT UART command definitions
#define _PID_SET_EP     0x00

// PPM Definitions
#define _PPM_BUF_SIZE           8  // 6 channels + 2 modes (manual drive and drive to manipulation switch)
//#define _I_PPM_BUF_MANUAL_MODE  0   //Index to dataBuf
#define _I_PPM_BUF_CTRL_MODE    0
#define _I_PPM_BUF_AUTO_MODE    1
#define _I_PPM_BUF_DATA_START   2   //Number of modes above

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

// UART Universal definitions
#define _I_UART_DATA_START      0

// UART_GO Definitions   (for receiving drive commands)
#define _UART_BUF_GO_SIZE       8   //6 speed bytes + 1 direction byte + 1 byte for CRC
#define _I_UART_GO_DIR          6   //Index for direction byte
#define _I_UART_GO_CRC          _UART_BUF_GO_SIZE - 1//7

// UART_AT definitions   (for receiving End Point Initialization)
//#define _UART_BUF_AT_SIZE       13  // [6 channels]*2 (2 byte values for each channel) + 1 byte CRC
//#define _I_UART_AT_CRC          _UART_BUF_AT_SIZE - 1//12
//#define _I_UART_AT_DATA_START   0

//PWM Definitions
#define _PWM_REG_SIZE           6

//PORT Definitions
#define _PORT_REG_SIZE          6


// Struct Definitions

struct PORT_Data {
    const uint8_t PORT_SIZE;
    //const uint8_t
    //TODO other stuff
    
    //PORT global parameters
    uint8_t iPort;  //incrimentor for port output
    bool frameEnd;
    
    //TMR5 (20ms Period Timer) functions  
    //StartFrame(); //Will implement in TMR5_Callback()
                    //called from TMR5_Callback()
                    //  sets framEnd = false
                    //  clears iPort
                    //  sets LATD = 0x01
                    //  loads TMR3 with reg[iPort]
    
    //void EndFrame(); //I think it belongs up here, not in PWM_Data
    
}portData = {
    _PORT_REG_SIZE, 
    0, 
    true};

/*PORT_Data functions*/
//void Init_PORT_Data(PORT_Data);

struct PWM_Data {
    
    const uint8_t PWM_REG_SIZE;
    //const uint8_t I_PWM_REG_DATA_START = _I_PWM_REG_DATA_START;
    
    uint16_t reg[_PWM_REG_SIZE];     //contains filtered data
    uint8_t iReg;
    //uint16_t buf[PWM_REG_SIZE];
    //uint8_t iBuf;
    //uint8_t ep_reg[2*PWM_REG_SIZE];    //end point register set (used for filtering buf data before entering reg)
    const uint16_t EP_ARRAY[2*_PWM_REG_SIZE];       //Full 180 degrees allowed on all channels
        //contains all endpoint values    
}pwmData = {
    _PWM_REG_SIZE,
    {
        _1_5MS_COMP,
        _1_5MS_COMP,
        _1_5MS_COMP,
        _1_5MS_COMP,
        _1_5MS_COMP,
        _1_5MS_COMP},
    0,
    {
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP}
    };

//void Init_PWM_Data();
//done in PORT_Data Struct: void EndFrame();    //Ends PWM frame at period clock interrupt
void UpdatePWM();//(UART_Data);   //filters/converts data from uart buf, and sends it to the pwm register
void UpdatePWM();//(PPM_Data);     //filters/converts data from ppm buf, and sends it to the pwm register

//private:
//void Convert(UART_Data* &uart); //Converts values to TMR3 usable values
//void Convert(PPM_Data* &ppm);
void Filter(); //cuts values at celing
//void UpdateReg(); //moves buf to reg



struct UART_Data {
    //GO command constants
    const uint8_t UART_BUF_SIZE;  //Size of Data buffer ("GO" is the PID)
    const uint8_t I_DIR;   //Direction byte 4'bxx000111' = rev,rev,rev,frw,frw,frw
    const uint8_t I_CRC;   //Cyclic redundancy check, this is a check-sum of data bytes only, (PID not included)
    const uint8_t I_UART_BUF_DATA_START;  //Start position of data (is 0)
    
    uint8_t buf[_UART_GO_BUF_SIZE];
    uint8_t iBuf;
    
    /* For setting endpoints. This option is on hold for now, and will be considered for future improvement
    //AT initialization command constants
    const uint8_t BUF_AT_SIZE = _UART_BUF_AT_SIZE;
    const uint8_t I_AT_DATA_START = _I_UART_AT_DATA_START;
    */
     
    /* Maybe later
    //RAM allocation for GO data
    uint8_t buf[BUF_AT_SIZE];     //contains byte data values sent via UART (used for GO and AT packets, notice that AT packets are larger)
    uint8_t iBuf;
    */
    
    /* Might use later
    //RAM allocation for AT data
    uint8_t ep_reg[BUF_AT_SIZE];
    */
    /*enum LoadState{READY, G_RECEIVED, O_RECEIVED, A_RECEIVED, 
                    T_RECEIVED, PID_GO_DRIVE_RECEIVED, PID_AT_SET_EP_RECEIVED};
   */
    enum LoadState{READY, G_RECEIVED, O_RECEIVED, PID_GO_DRIVE_RECEIVED};
    
    //state made into an enum: uint8_t loadState;     //holds the current state of data packet reception
    
    //bool goPacketReady;
    //bool atPacketReady;
}uartData = {
    _UART_BUF_GO_SIZE,
    _I_UART_GO_DIR,
    _I_UART_GO_CRC,
    _I_UART_DATA_START,
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    0};

//void Init_UART_Data();            //Constructor, (if not allowed in C, then call it explicitly)
bool CheckCRC();                    //Adds data bits, and compares with CRC byte
//void UpdateBuf(PPM_Data* &data);    //Translates data, and sends it to PPM_Data->buf;
//Update to PPM buffer happens externally
//bool IsGOPacketReady();   //Returns true if whole packet has been received
//bool IsATPacketReady();
void LoadByte(PPM_Data* &ppmMode, PWM_Data* &pwm);        //loads byte from Receive Register


    
}

struct PPM_Data {
    const uint8_t PPM_BUF_SIZE = _PPM_BUF_SIZE;                 //PPM buffer size
    //const uint8_t I_MANUAL_MODE = _I_PPM_BUF_MANUAL_MODE;
    const uint8_t I_CTRL_MODE = _I_PPM_BUF_CTRL_MODE;   //_2MS for drive
    const uint8_t I_AUTO_MODE = _I_PPM_BUF_AUTO_MODE;
    const uint8_t I_PPM_BUF_DATA_START = _I_PPM_BUF_DATA_START;
    
    const enum LoadState{READY, BREAK_RECEIVED};    //tracks the current state of the PPM data algorithm
    
    uint16_t buf[PPM_BUF_SIZE];     //contains timer values as received
    uint8_t iBuf;               //Buffer index counter

    //bool ppmValid;
    
    void Init_PPM_Data();         //Constructor (if allowed in C)
    //size_t state;         //tracks the state of the PPM input (0: waiting for break pulse, 1: break pulse received)
    void PPMRead(PWM_Data* &pwm);     //Sends captured CCP1 value to PPM buffer
    
    //states/modes
    //bool IsPPMValid();    //returns true if whole frame was received successfully, and has not yet been written over
    bool IsManualMode();    //returns true if in manual control mode (overrides autonomous control)
    bool IsManipulationMode();     //returns true if in manipulation mode
    bool IsAutoMode();      //returns true if autonomy is enabled AND manual mode is disabled
    
    bool IsDriveCont();     //returns true if the controller is used as rover motor driver, and false if it is used as Manipulation controller
    bool IsUARTMode();      //returns true if the state is set for using UART data to drive the controller
    bool IsPPMMode();       //returns true if the state is set for using PPM data to drive the controller
    
    bool GetAutoModeState();    //Interprets and returns mode setting sent over PPM
    bool GetCtrlModeState();    //Interprets and returns mode setting sent over PPM
    
    //void UpdateReg(PWM_Data* &pwm);       //updates the main register (updates from its own data buffer when in manual mode,
    //void UpdateReg(UART_Data* &uart, PWM_Data* &pwm);//  or updates from UART buffer when in autonomous mode)
}ppmData;

