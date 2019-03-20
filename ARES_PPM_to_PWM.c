//*
// * File:   ARES_PPM_to_PWM.c
// * Author: Derek
// * 
// * A few equations from the datasheet, assuming a clock source Fosc/4:
// *  PWM Period = [(TxPR)+1]*4*Tosc*TMR_Prescale
// *  Pulse Width = CCPRxH:CCPRxL*Tosc*TMR_Prescale
// *  Duty Cycle Ratio = (CCPRxH:CCPRxL)/[4(TxPR+1)]
// *  Resolution = log[4(TxPR+1)]/log(2) bits
// *  See page 317 of the datasheet for further details
// * 
// *  Baud rate (Page 487 of datasheet)
// *      Fclock = Fosc/[4(SSPxADD + 1)]
// * 
// * Talon SRX requirements:
// * 
// *  PWM Period:
// *      Min: 2.9 ms
// *      Max: 100 ms
// *      Typical: 20 ms
// *      Decided: 10 ms to increase control resolution
// * 
// *  High Pulse:
// *      Min: 1 ms (reverse)
// *      Max: 2 ms (forward)
// *      Mid: 1.5 ms (Stopped)
// *
// * Created on March 12, 2019, 5:09 PM
// */

#include "main.h"

//#define HIGH_PULSE  _2MS_COMP
//#define MID_PULSE   _1_5MS_COMP
//#define LOW_PULSE   _1MS_COMP
//#define UART_MAX    0xFF
//#define UART_MIN    0x00;

static const float UART_CONVERSION_MULTIPLIER_HIGH  =  -7.843137255;     //calculated from (_2MS_COMP - _1_5MS_COMP)/255
static const float UART_CONVERSION_MULTIPLIER_LOW  =    7.843137255; //calculated from (_1MS_COMP - _1_5MS_COMP)/255
static const float OFFSET = 59536;   //=_1_5MS_COMP

//Initialize Structs
//Declare Instances
volatile struct PORT_Data portData = {_PORT_REG_SIZE};


volatile struct PWM_Data pwmData = {
    _PWM_REG_SIZE, 
    {       //Full 180 degrees allowed on all channels
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP,
        _1MS_COMP, _2MS_COMP
    }  //contains all endpoint values};
};

volatile struct UART_Data uartData = {
    _UART_BUF_GO_SIZE,  //Size of Data buffer ("GO" is the PID)
    _I_UART_GO_DIR,   //Direction byte 4'bxx000111' = rev,rev,rev,frw,frw,frw
    _I_UART_GO_CRC,   //Cyclic redundancy check, this is a check-sum of data bytes only, (PID not included)
    _I_UART_DATA_START  //Start position of data (is 0)
};

volatile struct PPM_Data ppmData = {
    _PPM_BUF_SIZE,                 //PPM buffer size
    //_I_PPM_BUF_MANUAL_MODE,
    _I_PPM_BUF_CTRL_MODE,   //_2MS for drive
    _I_PPM_BUF_AUTO_MODE,
    _I_PPM_BUF_DATA_START
};

volatile enum UARTLoadState uartLoadState = UART_READY;
volatile enum PPMLoadState ppmLoadState = PPM_READY;

//struct PORT_Data GetPortData() {return portData;}
//struct PWM_Data GetPwmData() {return pwmData;}
//struct UART_Data GetUartData() {return uartData;}
//struct PPM_Data GetPpmData() {return ppmData;}

//PORT_Data struct
void Init_PORT_Data(struct PORT_Data *port) {
    port->iPort = 0;
    port->frameEnd = true;    //must wait for break pulse before sending PWM pulses
}

//PWM_Data struct functions
void Init_PWM_Data(struct PWM_Data *pwm) {
    for(uint8_t i = 0; i < pwm->PWM_REG_SIZE; i++) {
        pwm->reg[i] = _1_5MS_COMP;   //initialize at center position (or stop position)
    }
    pwm->iReg = 0; //indecies used in case of interrupt triggered increment
}

uint16_t Filter(struct PWM_Data *pwm, uint16_t temp, uint8_t i) {
    if(temp > pwm->EP_ARRAY[2*i]) temp = pwm->EP_ARRAY[2*i];
    else if(temp < pwm->EP_ARRAY[2*i + 1]) temp = pwm->EP_ARRAY[2*i + 1];
    return temp;
}

void UARTUpdatePWM(struct PWM_Data *pwm, struct UART_Data *uart) {
    
    //Convert, and place in buf[]
    uint8_t dir_reg = uart->buf[uart->I_DIR];
    for(uint8_t i = 0; i < pwm->PWM_REG_SIZE; i++) {
        //if(_PIC_IS_DRIVE_CONT == true) {    //if this the PIC is the drive controller, 
                                    //  then the 3 last channels are reversed
                                    //  data interpretation is also different
                                    //  but as for now, no manipulation data should be coming through uart
        uint8_t dir = dir_reg&0x01; //get the direction for the ith channel
        dir_reg = dir_reg >> 1; //then shift the direction register down to get the next direction bit
        //const float HIGH_PULSE = ((uint16_t)_2MS_COMP);
        //const float MID_PULSE = ((uint16_t)_1_5MS_COMP);
        //const float LOW_PULSE = ((uint16_t)_1MS_COMP);
        //const float UART_MAX = ((uint16_t)0x00FF);
        //const uint8_t UART_MIN = 0x00;
        //uint16_t temp = 0;  //using temporary uint16_t rather than a secondary buffer
        //if(((i < 3)&&(dir == _DIR_FORWARD))||((i >= 3)&&(dir == _DIR_REVERSE))) {
        //    temp = ((uint16_t)((float)uart->buf[i]*(HIGH_PULSE - MID_PULSE)/UART_MAX) + MID_PULSE);
        //}
        //else {
        //    temp = ((uint16_t)((float)uart->buf[i]*(LOW_PULSE - MID_PULSE)/UART_MAX) + MID_PULSE);
        //}
        
        //second attempt at above equastions:
        uint16_t temp = uart->buf[i];
        
        if(((i < 3)&&(dir == _DIR_FORWARD))||((i >= 3)&&(dir == _DIR_REVERSE))) {
            temp = temp*UART_CONVERSION_MULTIPLIER_HIGH + OFFSET;
        }
        else {
            temp = temp*UART_CONVERSION_MULTIPLIER_LOW + OFFSET;
        }
        
        //filter and place temp in reg[i]
        pwm->reg[i] = Filter(pwm, temp, i);
    }
}

void PPMUpdatePWM(struct PWM_Data *pwm, struct PPM_Data *ppm) {
    for(uint8_t i = 0; i < pwm->PWM_REG_SIZE; i++) {
        pwm->reg[i] = Filter(pwm, ~ppm->buf[i+ppm->I_PPM_BUF_DATA_START], i);   //complement of ppm (time period) gives starting time for TMR3
    }
}

//PPM_Data struct functions
void Init_PPM_Data(struct PPM_Data *ppm) {
//    buf[I_MANUAL_MODE] = _1MS;     //Manual mode is disabled by default
    if(_PIC_IS_DRIVE_CONT == true) ppm->buf[ppm->I_CTRL_MODE] = _2MS;      //Drive mode is selected by default (not manipulation mode)
    else ppm->buf[ppm->I_CTRL_MODE] = _1MS;
    ppm->buf[ppm->I_AUTO_MODE] = _1MS;      //Autonomous Mode is disabled by default
    for (uint8_t i = ppm->I_PPM_BUF_DATA_START; i < ppm->PPM_BUF_SIZE; i++) {
        ppm->buf[i] = _1_5MS;    //1.5 ms as default (0 position)
    }
    ppm->iBuf = 0;   //buffer index
    
    ppmLoadState = PPM_READY;
}

void PPMRead(struct PPM_Data *ppm, struct PWM_Data *pwm) {
    if(CCP1_IsCapturedDataReady() == true) {
        switch(ppmLoadState) {
            case PPM_READY:
                ppm->iBuf = 0;
                if(CCP1_CaptureRead() >= _6MS) {   //assuming a break pulse to be at least 6ms (knowing the _6MS_COMP is the compliment value)
                    ppmLoadState = BREAK_RECEIVED;
                }
                break;
            case BREAK_RECEIVED:
                if(ppm->iBuf < ppm->PPM_BUF_SIZE) {
                    ppm->buf[ppm->iBuf] = CCP1_CaptureRead();
                    ppm->iBuf++;
                    if(ppm->iBuf >= ppm->PPM_BUF_SIZE) {
                        if(IsPPMMode(ppm) == true) {
                            //ppmValid = true;   //PPM frame has been loaded, set status to valid
                            PPMUpdatePWM(pwm, ppm);
                        }
                        ppmLoadState = PPM_READY;
                    }
                }
                else {                  //if the index is out of bounds, then data is likely invalid
                    //ppmValid = false;
                    ppmLoadState = PPM_READY;
                }
                break;
            default:
                //ppmValid = false;   //if entering default, then the data is likely invalid
                                    //  Likely the PPM break was never detected
                ppmLoadState = PPM_READY;
        }
        
    }
    
}

bool GetAutoModeState(struct PPM_Data *ppm) {
    if(ppm->buf[ppm->I_AUTO_MODE] < _1_5MS) return false;
    else return true;
}

bool GetCtrlModeState(struct PPM_Data *ppm) {
    if(ppm->buf[ppm->I_CTRL_MODE] < _1_5MS) return false;
    else return true;
}

bool IsAutoMode(struct PPM_Data *ppm) {
    if((GetAutoModeState(ppm) == true)&&(GetCtrlModeState(ppm) == true)) return true;
    else return false;
}

bool IsManualMode(struct PPM_Data *ppm) {
    if((GetAutoModeState(ppm) == false)&&(GetCtrlModeState(ppm) == true)) return true;
    else return false;
}

bool IsManipulationMode(struct PPM_Data *ppm) {
    if(GetCtrlModeState(ppm) == false) return true;
    else return false;
}

bool IsDriveCont() {  //might upgrade this to be configurable over UART, and saved in EEPROM (PIC16F1777 has no EEPROM storage)
    return _PIC_IS_DRIVE_CONT;
}

bool IsUARTMode(struct PPM_Data *ppm) {
    if((IsDriveCont() == true)&&(IsAutoMode(ppm) == true)) return true;
    else return false;
}

bool IsPPMMode(struct PPM_Data *ppm) {
    if(((IsDriveCont() == true)&&(IsManualMode(ppm) == true))
            ||((IsDriveCont() == false)&&IsManipulationMode(ppm))) {
        return true;
    }
    else return false;
}

// UART_Data struct functions 
void Init_UART_Data(struct UART_Data *uart) {
    for (uint8_t i = uart->I_UART_BUF_DATA_START; i < uart->UART_BUF_SIZE; i++) {
        uart->buf[i] = 0;    //set all defaults at 0 position, CRC is also 0
    }
    
    uart->iBuf = 0;   //buffer index
}

bool CheckCRC(struct UART_Data *uart) {
    uint8_t inc = 0;
    for(uint8_t i = uart->I_UART_BUF_DATA_START; i < uart->I_CRC; i++) {
        for(uint8_t j = 0; j < 8; j++) {
            inc = inc + ((uart->buf[i] >> j)&0x01);   //add up the number of bits
        }
    }
    if(inc == uart->buf[uart->I_CRC]) return true;
    else return false;
}

void LoadByte(struct UART_Data *uart, struct PPM_Data *ppmMode, struct PWM_Data *pwm) {
    if(PIR1bits.RCIF == 0) return;  //safe to place this in the function.
                                    //If the flag is clear, then do not try reading UART
//    if(PIR1bits.RCIF == 1) {
//        //PIR1bits.RCIF = 0;  //clear the UART receive interrupt flag
//        //RCREG must be read to clear RCIF
//    }
    
    uint8_t byte;   //temporary storage for uart read
    
    switch(uartLoadState) {
        case UART_READY:
            byte = EUSART_Read();           //two state option to branch to
            if(byte == 'G') uartLoadState = G_RECEIVED;
            //else if(byte == 'A') LoadState = A_RECEIVED;
            break;
        case G_RECEIVED:
            if(EUSART_Read() == 'O') uartLoadState = O_RECEIVED;
            else uartLoadState = UART_READY;    //revert back to READY if 'O' was not received consecutively
            break;
        case O_RECEIVED:
            if((EUSART_Read() == _PID_DRIVE)&&(IsUARTMode(ppmMode) == true)) { //0 byte at start of frame
                uartLoadState = PID_GO_DRIVE_RECEIVED;
                uart->iBuf = 0;   //initialize the buffer pointer
            }
            else uartLoadState == UART_READY;    //revert back to READY if 0x00 was not received consecutively
            break;
        //case A_RECEIVED:
        //    if(EUSART_Read() == 'T') LoadState = T_RECEIVED;
        //    else LoadState == READY;    //revert back to READY if 'T' was not received consecutively
        //    break;
        //case T_RECEIVED:
        //    if(EUSART_Read() == _PID_SET_EP) { //0 byte at start of frame
        //        LoadState = PID_AT_SET_EP_RECEIVED;
        //        iBuf = 0;   //initialize the buffer pointer
        //    }
        //    else LoadState == READY;    //revert back to READY if 0x00 was not received consecutively
        //    break;
        case PID_GO_DRIVE_RECEIVED:
            uart->buf[uart->iBuf] = EUSART_Read();
            uart->iBuf++;
            if(uart->iBuf >= uart->UART_BUF_SIZE) {
                if(CheckCRC(uart) == true) UARTUpdatePWM(pwm, uart); //update the pwm register if UART check cum was successful
                uartLoadState = UART_READY;
                //goPacketReady = true;   //NOTE: make sure the buffer is read before goPacketReady = false
            }
            break;
        //case PID_AT_SET_EP_RECEIVED:
        //    buf[iBuf] = EUSART_Read();
        //    iBuf++;
        //    if(iBuf >= BUF_AT_SIZE) {
        //        if(CheckCRC() == true) pwmData.UpdateReg(uartData);
        //        !!!//need to get data size in packet, or something to know where the end of packet is 
        //        LoadState = READY;
        //        //atPacketReady = true;   //NOTE: make sure the buffer is read before atPacketReady = false
        //    }
        //    break;
        default:
            //EUSART_Read();  //clear the FIFO if data comes in out of place
            uartLoadState = UART_READY; //reset state to READY if an unknown state was reached
            
    }
}
