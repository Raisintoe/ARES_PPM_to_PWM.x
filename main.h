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

#define _1MS_COMP       0xF060  //1ms as interpreted by timer1 and timer3
#define _1_5MS_COMP     0xE890  //1.5ms as interpreted by timer1 and timer3
#define _2MS_COMP       0xE0C0  //2ms as interpreted by timer1 and timer3
#define _4MS_COMP       0xC180
#define _6MS_COMP       0xA240  //For Detecting Break Pulse

#define _1MS            ~_1MS_COMP // or 0xFFFF - _1MS_COMP
#define _1_5MS          ~_1_5MS_COMP // or 0xFFFF - _1_5MS_COMP
#define _2MS            ~_2MS_COMP // or 0xFFFF - _2MS_COMP
#define _4MS            ~_4MS_COMP // or 0xFFFF - _4MS_COMP
#define _6MS            ~_6MS_COMP // or 0xFFFF - _6MS_COMP
#define _RESOLUTION     _2MS-_1MS

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

//PORT_Data Struct
struct PORT_Data {
    const uint8_t PORT_SIZE;// = _PORT_REG_SIZE;
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
    
    
};//portData;
    

//PWM_Data Struct
struct PWM_Data {
    
    const uint8_t PWM_REG_SIZE;// = _PWM_REG_SIZE;
    //const uint8_t I_PWM_REG_DATA_START = _I_PWM_REG_DATA_START;
    const uint16_t EP_ARRAY[2*_PWM_REG_SIZE];// = {       //Full 180 degrees allowed on all channels
    //    _1MS_COMP, _2MS_COMP,
    //    _1MS_COMP, _2MS_COMP,
    //    _1MS_COMP, _2MS_COMP,
    //    _1MS_COMP, _2MS_COMP,
    //    _1MS_COMP, _2MS_COMP,
    //    _1MS_COMP, _2MS_COMP
    //};  //contains all endpoint values
    
    
    uint16_t reg[_PWM_REG_SIZE];     //contains filtered data
    uint8_t iReg;
    //uint16_t buf[PWM_REG_SIZE];
    //uint8_t iBuf;
    //uint8_t ep_reg[2*PWM_REG_SIZE];    //end point register set (used for filtering buf data before entering reg)
    
    
};//pwmData;


//UART_Data Struct
struct UART_Data {
    //GO command constants
    const uint8_t UART_BUF_SIZE;// = _UART_BUF_GO_SIZE;  //Size of Data buffer ("GO" is the PID)
    const uint8_t I_DIR;// = _I_UART_GO_DIR;   //Direction byte 4'bxx000111' = rev,rev,rev,frw,frw,frw
    const uint8_t I_CRC;// = _I_UART_GO_CRC;   //Cyclic redundancy check, this is a check-sum of data bytes only, (PID not included)
    const uint8_t I_UART_BUF_DATA_START;// = _I_UART_DATA_START;  //Start position of data (is 0)
    
    uint8_t buf[_UART_BUF_GO_SIZE];
    uint8_t iBuf;
    
    
    
    // For setting endpoints. This option is on hold for now, and will be considered for future improvement
    //AT initialization command constants
    //const uint8_t BUF_AT_SIZE = _UART_BUF_AT_SIZE;
    //const uint8_t I_AT_DATA_START = _I_UART_AT_DATA_START;
    
     
    // Maybe later
    //RAM allocation for GO data
    //uint8_t buf[BUF_AT_SIZE];     //contains byte data values sent via UART (used for GO and AT packets, notice that AT packets are larger)
    //uint8_t iBuf;
    
    
    // Might use later
    //RAM allocation for AT data
    //uint8_t ep_reg[BUF_AT_SIZE];
    
    //enum LoadState{READY, G_RECEIVED, O_RECEIVED, A_RECEIVED, 
    //                T_RECEIVED, PID_GO_DRIVE_RECEIVED, PID_AT_SET_EP_RECEIVED};
   
    
    
    //bool goPacketReady;
    //bool atPacketReady;
};//uartData;

enum UARTLoadState{UART_READY, G_RECEIVED, O_RECEIVED, PID_GO_DRIVE_RECEIVED};

struct PPM_Data {
    const uint8_t PPM_BUF_SIZE;// = _PPM_BUF_SIZE;                 //PPM buffer size
    //const uint8_t I_MANUAL_MODE = _I_PPM_BUF_MANUAL_MODE;
    const uint8_t I_CTRL_MODE;// = _I_PPM_BUF_CTRL_MODE;   //_2MS for drive
    const uint8_t I_AUTO_MODE;// = _I_PPM_BUF_AUTO_MODE;
    const uint8_t I_PPM_BUF_DATA_START;// = _I_PPM_BUF_DATA_START;
    
    
    
    uint16_t buf[_PPM_BUF_SIZE];     //contains timer values as received
    uint8_t iBuf;               //Buffer index counter

    //bool ppmValid;
};//ppmData;

enum PPMLoadState{PPM_READY, BREAK_RECEIVED};    //tracks the current state of the PPM data algorithm

//Struct Functions

//PPM Functions
void Init_PPM_Data(struct PPM_Data *ppm);         //Constructor (if allowed in C)
//size_t state;         //tracks the state of the PPM input (0: waiting for break pulse, 1: break pulse received)
void PPMRead(struct PPM_Data *ppm, struct PWM_Data *pwm);     //Sends captured CCP1 value to PPM buffer

//states/modes
//bool IsPPMValid();    //returns true if whole frame was received successfully, and has not yet been written over
bool IsManualMode(struct PPM_Data *ppm);    //returns true if in manual control mode (overrides autonomous control)
bool IsManipulationMode(struct PPM_Data *ppm);     //returns true if in manipulation mode
bool IsAutoMode(struct PPM_Data *ppm);      //returns true if autonomy is enabled AND manual mode is disabled

bool IsDriveCont();     //returns true if the controller is used as rover motor driver, and false if it is used as Manipulation controller
bool IsUARTMode(struct PPM_Data *ppm);      //returns true if the state is set for using UART data to drive the controller
bool IsPPMMode(struct PPM_Data *ppm);       //returns true if the state is set for using PPM data to drive the controller

bool GetAutoModeState(struct PPM_Data *ppm);    //Interprets and returns mode setting sent over PPM
bool GetCtrlModeState(struct PPM_Data *ppm);    //Interprets and returns mode setting sent over PPM

//void UpdateReg(PWM_Data* &pwm);       //updates the main register (updates from its own data buffer when in manual mode,
//void UpdateReg(UART_Data* &uart, PWM_Data* &pwm);//  or updates from UART buffer when in autonomous mode)





//UART Functions
void Init_UART_Data(struct UART_Data *uart);            //Constructor, (if not allowed in C, then call it explicitly)
bool CheckCRC(struct UART_Data *uart);                    //Adds data bits, and compares with CRC byte
//void UpdateBuf(PPM_Data* &data);    //Translates data, and sends it to PPM_Data->buf;
//Update to PPM buffer happens externally
//bool IsGOPacketReady();   //Returns true if whole packet has been received
//bool IsATPacketReady();
void LoadByte(struct UART_Data *uart, struct PPM_Data *ppmMode, struct PWM_Data *pwm);        //loads byte from Receive Register
//state made into an enum: uint8_t loadState;     //holds the current state of data packet reception



//PWM Functions
void Init_PWM_Data(struct PWM_Data *pwm);
//done in PORT_Data Struct: void EndFrame();    //Ends PWM frame at period clock interrupt
void UARTUpdatePWM(struct PWM_Data *pwm, struct UART_Data *uart);   //filters/converts data from uart buf, and sends it to the pwm register
void PPMUpdatePWM(struct PWM_Data *pwm, struct PPM_Data *ppm);     //filters/converts data from ppm buf, and sends it to the pwm register
    
//private:
//void Convert(UART_Data* &uart); //Converts values to TMR3 usable values
//void Convert(PPM_Data* &ppm);
uint16_t Filter(struct PWM_Data *pwm, uint16_t temp, uint8_t i); //cuts values at celing
//void UpdateReg(); //moves buf to reg

//Port Data Functions
void Init_PORT_Data(struct PORT_Data *port);
//void EndFrame(); //I think it belongs up here, not in PWM_Data


//Needed for TMR3 and TMR5 CallBack() functions
//struct PORT_Data GetPortData();
//struct PWM_Data GetPwmData();
//struct UART_Data GetUartData();
//struct PPM_Data GetPpmData();