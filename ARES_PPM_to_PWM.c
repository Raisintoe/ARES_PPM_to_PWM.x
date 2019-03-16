/*
 * File:   ARES_PPM_to_PWM.c
 * Author: Derek
 * 
 * A few equations from the datasheet, assuming a clock source Fosc/4:
 *  PWM Period = [(TxPR)+1]*4*Tosc*TMR_Prescale
 *  Pulse Width = CCPRxH:CCPRxL*Tosc*TMR_Prescale
 *  Duty Cycle Ratio = (CCPRxH:CCPRxL)/[4(TxPR+1)]
 *  Resolution = log[4(TxPR+1)]/log(2) bits
 *  See page 317 of the datasheet for further details
 * 
 *  Baud rate (Page 487 of datasheet)
 *      Fclock = Fosc/[4(SSPxADD + 1)]
 * 
 * Talon SRX requirements:
 * 
 *  PWM Period:
 *      Min: 2.9 ms
 *      Max: 100 ms
 *      Typical: 20 ms
 *      Decided: 10 ms to increase control resolution
 * 
 *  High Pulse:
 *      Min: 1 ms (reverse)
 *      Max: 2 ms (forward)
 *      Mid: 1.5 ms (Stopped)
 *
 * Created on March 12, 2019, 5:09 PM
 */

#include "main.h"

//Init_PORT_Data(PORT_Data port) {
//    port.PORT_SIZE = _PORT_REG_SIZE;
//    port.iPort = 0;
//    port.frameEnd = true;    //must wait for break pulse before sending PWM pulses
//}

/*PWM_Data struct functions*/
/*PWM_Data::Init_PWM_Data() {
    for(uint8_t i = 0; i < PWM_REG_SIZE; i++) {
        reg[i] = _1_5MS_COMP;   //initialize at center position (or stop position)
    }
    iReg = 0; //indecies used in case of interrupt triggered increment
}
*/

PWM_Data::UpdatePWM(UART_Data* &uart) {
    
    //Convert, and place in buf[]
    for(uint8_t i = 0; i < PWM_REG_SIZE; i++) {
        //if(_PIC_IS_DRIVE_CONT) {    //if this the PIC is the drive controller, 
                                    //  then the 3 last channels are reversed
                                    //  data interpretation is also different
                                    //  but as for now, no manipulation data should be coming through uart
        uint8_t dir = (uart->buf[uart->I_DIR] >> i)&0x01;
        const uint16_t HIGH_PULSE = _2MS_COMP;
        const uint16_t MID_PULSE = _1_5MS_COMP;
        const uint16_t LOW_PULSE = _1MS_COMP;
        const uint8_t UART_MAX = 0xFF;
        //const uint8_t UART_MIN = 0x00;
        uint16_t temp = 0;  //using temporary uint16_t rather than a secondary buffer
        if(((i < 3)&&(dir == _DIR_FORWARD))||((i >= 3)&&(dir == _DIR_REVERSE))) {
            temp = ((uint16_t)((double)uart->buf[i]*(HIGH_PULSE - MID_PULSE)/UART_MAX) + MID_PULSE);
        }
        else {
            temp = ((uint16_t)((double)uart->buf[i]*(LOW_PULSE - MID_PULSE)/UART_MAX) + MID_PULSE);
        }
        
        //filter and place temp in reg[i]
        reg[i] = Filter(temp, i);
    }
}

PWM_Data::UpdatePWM(PPM_Data* &ppm) {
    for(uint8_t i = 0; i < PWM_REG_SIZE; i++) {
        reg[i] = Filter(ppm->buf[i], i);
    }
}

uint16_t PWM_Data::Filter(uint16_t temp, uint8_t i) {
    if(temp > EP_ARRAY[2*i]) temp = EP_ARRAY[2*i];
    else if(temp < EP_ARRAY[2*i + 1]) temp = EP_ARRAY[2*i + 1];
    return temp;
}


/*PPM_Data struct functions*/
PPM_Data::Init_PPM_Data() {
//    buf[I_MANUAL_MODE] = _1MS_COMP;     //Manual mode is disabled by default
    if(_PIC_IS_DRIVE_CONT) buf[I_CTRL_MODE] = _2MS_COMP;      //Drive mode is selected by default (not manipulation mode)
    else buf[I_CTRL_MODE] = _1MS_COMP;
    buf[I_AUTO_MODE] = _1MS_COMP;      //Autonomous Mode is disabled by default
    for (uint8_t i = I_PPM_BUF_DATA_START; i < PPM_BUF_SIZE; i++) {
        buf[i] = _1_5MS_COMP;    //1.5 ms as default (0 position)
    }
    iBuf = 0;   //buffer index
    
    LoadState = READY;
}

PPM_Data::PPMRead(PWM_Data* &pwm) {
    if(CCP1_IsCapturedDataReady()) {
        switch(LoadState) {
            case READY:
                iBuf = 0;
                if(CCP1_CaptureRead() <= _6MS_COMP) {   //assuming a break pulse to be at least 6ms (knowing the _6MS_COMP is the compliment value)
                    LoadState = BREAK_RECEIVED;
                }
                break;
            case BREAK_RECEIVED:
                if(iBuf < PPM_BUF_SIZE) {
                    buf[iBuf] = CCP1_CaptureRead();
                    iBuf++;
                    if(iBuf >= PPM_BUF_SIZE) {
                        if(IsPPMMode()) {
                            //ppmValid = true;   //PPM frame has been loaded, set status to valid
                            pwm->UpdatePWM(Init_PPM_Data);
                        }
                        LoadState = READY;
                    }
                }
                else {                  //if the index is out of bounds, then data is likely invalid
                    //ppmValid = false;
                    LoadState = READY;
                }
                break;
            default:
                //ppmValid = false;   //if entering default, then the data is likely invalid
                                    //  Likely the PPM break was never detected
                LoadState = READY;
        }
        
    }
    
}

bool PPM_Data::GetAutoModeState() {
    if(buf[I_AUTO_MODE] > _1_5MS_COMP) return false;
    else return true;
}

bool PPM_Data::GetCtrlModeState() {
    if(buf[I_CTRL_MODE] > _1_5MS_COMP) return false;
    else return true;
}

bool PPM_Data::IsAutoMode() {
    if(GetAutoModeState()&&GetCtrlModeState()) return true;
    else return false;
}

bool PPM_Data::IsManualMode() {
    if((!GetAutoModeState())&&GetCtrlModeState()) return true;
    else return false;
}

bool PPM_Data::IsManipulationMode() {
    if(!GetCtrlModeState()) return true;
    else return false;
}

bool PPM_Data::IsDriveCont() {  //might upgrade this to be configurable over UART, and saved in EEPROM (PIC16F1777 has no EEPROM storage)
    return _PIC_IS_DRIVE_CONT;
}

bool PPM_Data::IsUARTMode() {
    if(IsDriveCont()&&IsAutoMode()) return true;
    else return false;
}

bool PPM_Data::IsPPMMode() {
    if((IsDriveCont()&&IsManualMode())
            ||(!IsDriveCont()&&IsManipulationMode())) {
        return true;
    }
    else return false;
}

/* UART_Data struct functions */
/*UART_Data::Init_UART_Data() {
    for (uint8_t i = I_UART_BUF_DATA_START; i < UART_BUF_SIZE; i++) {
        buf[i] = 0;    //set all defaults at 0 position, CRC is also 0
    }
    
    iBuf = 0;   //buffer index
}
*/
bool UART_Data::CheckCRC() {
    uint8_t inc = 0;
    for(uint8_t i = I_UART_BUF_DATA_START; i < I_CRC; i++) {
        for(uint8_t j = 0; j < 8; j++) {
            inc = inc + ((buf[i] >> j)&0x01);   //add up the number of bits
        }
    }
    if(inc == buf[I_CRC]) return true;
    else return false;
}

UART_Data::LoadByte(PPM_Data* &ppmMode, PWM_Data* &pwm) {
    if(PIR1bits.RCIF == 1) {
        //PIR1bits.RCIF = 0;  //clear the UART receive interrupt flag
        //RCREG must be read to clear RCIF
    }
    
    switch(LoadState) {
        case READY:
            uint8_t byte = EUSART_Read();           //two state option to branch to
            if(byte == 'G') LoadState = G_RECEIVED;
            //else if(byte == 'A') LoadState = A_RECEIVED;
            break;
        case G_RECEIVED:
            if(EUSART_Read() == 'O') LoadState = O_RECEIVED;
            else LoadState == READY;    //revert back to READY if 'O' was not received consecutively
            break;
        case O_RECEIVED:
            if((EUSART_Read() == _PID_DRIVE)&&ppmMode->IsUARTMode()) { //0 byte at start of frame
                LoadState = PID_GO_DRIVE_RECEIVED;
                iBuf = 0;   //initialize the buffer pointer
            }
            else LoadState == READY;    //revert back to READY if 0x00 was not received consecutively
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
            buf[iBuf] = EUSART_Read();
            iBuf++;
            if(iBuf >= UART_BUF_SIZE) {
                if(CheckCRC()) pwm->UpdatePWM(Init_UART_Data); //update the pwm register if UART check cum was successful
                LoadState = READY;
                //goPacketReady = true;   //NOTE: make sure the buffer is read before goPacketReady = false
            }
            break;
        //case PID_AT_SET_EP_RECEIVED:
        //    buf[iBuf] = EUSART_Read();
        //    iBuf++;
        //    if(iBuf >= BUF_AT_SIZE) {
        //        if(CheckCRC()) pwmData.UpdateReg(uartData);
        //        !!!//need to get data size in packet, or something to know where the end of packet is 
        //        LoadState = READY;
        //        //atPacketReady = true;   //NOTE: make sure the buffer is read before atPacketReady = false
        //    }
        //    break;
        default:
            //EUSART_Read();  //clear the FIFO if data comes in out of place
            LoadState = READY; //reset state to READY if an unknown state was reached
            
    }
}
