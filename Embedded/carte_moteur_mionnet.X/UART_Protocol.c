#include <stdio.h>
#include <stdlib.h>
#include <libpic30.h>
#include "timer.h"
#include <xc.h>
#include "PWM.h"
#include "Robot.h"
#include "ADC.h"
#include "UART_Protocol.h"
#include "UART.h"
#include "CB_TX1.h"
#include "IO.h"
#include "main.h"
#include "CB_RX1.h"
#include "automatique.h"

unsigned char boucle;

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char * msgPayload) {
    
    //Fonction prenant entr�e la trame et sa longueur pour calculer le checksum
    unsigned char checksum;
    checksum = 0xFE;
    checksum ^= (char) (msgFunction >> 8);
    checksum ^= (char) (msgFunction >> 0);
    checksum ^= (char) (msgPayloadLength >> 8);
    checksum ^= (char) (msgPayloadLength >> 0);
    int i = 0;
    for (i = 0; i < msgPayloadLength; i++) {
        checksum ^= msgPayload[i];
    }
    return checksum;
}

void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    
    unsigned char checksum;
    unsigned char msg[6 + msgPayloadLength];
    int pos = 0;
    msg[pos++] = 0xFE;
    msg[pos++] = (msgFunction >> 8);
    msg[pos++] = (msgFunction >> 0);
    msg[pos++] = (msgPayloadLength >> 8);
    msg[pos++] = (msgPayloadLength >> 0);
    int i = 0;
    for (i = 0; i < msgPayloadLength; i++) {
        msg[pos++] = msgPayload[i];
    }
    checksum = UartCalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
    msg[pos++] = checksum;
    SendMessage(msg, pos);

}

int msgDecodedFunction;
int msgDecodedPayloadLength;
unsigned char msgDecodedPayload[128];
int msgDecodedPayloadIndex;
int calculatedCheckSum;
int receivedCheckSum;

int static rcvState;

void UartDecodedMessage(unsigned char c) {
    

    switch (rcvState) {

        case Waiting:
            
            if (c == 0xFE) {
                
                msgDecodedFunction = 0;
                msgDecodedPayloadLength = 0;
                msgDecodedPayloadIndex = 0;
                rcvState = Function_MSB;
            }
            break;

        case Function_MSB:
            
            msgDecodedFunction = c;
            msgDecodedFunction = msgDecodedFunction << 8;
            rcvState = Function_LSB;
            break;

        case Function_LSB:
            msgDecodedFunction += c;
            rcvState = PayloadLength_MSB;
            break;

        case PayloadLength_MSB:
            msgDecodedPayloadLength = c;
            msgDecodedPayloadLength = msgDecodedPayloadLength << 8;
            rcvState = PayloadLength_LSB;
            break;

        case PayloadLength_LSB:
            msgDecodedPayloadLength += c;
            if (msgDecodedPayloadLength == 0) {
                rcvState = Waiting;
            } else {
                rcvState = Payload;
            }
            break;

        case Payload:
            msgDecodedPayload [msgDecodedPayloadIndex] = (char) c;
            msgDecodedPayloadIndex++;
            if (msgDecodedPayloadIndex == msgDecodedPayloadLength) {
                rcvState = CheckSum;
            }
            break;

        case CheckSum:
            receivedCheckSum = c;
            calculatedCheckSum = UartCalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            if (calculatedCheckSum == receivedCheckSum) {
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                
            }
            rcvState = Waiting;
            break;

        default:
            rcvState = Waiting;
            break;
    }

    //Fonction appel�e apr�s le d�codage pour ex�cuter l?action
    //correspondant au message re�u
    //?
}



//*************************************************************************/
//Fonctions correspondant aux messages
//*************************************************************************/

void SetRobotState(unsigned char * payload) {

}

void SetRobotAutoControlState(unsigned char * payload) {
    boucle = payload[0];
    robotState.isAutoControl = boucle;
    if (robotState.isAutoControl == 0x01) {
        robotState.isAutoControl = TRUE;
//         LED_ORANGE=0;
//            LED_BLANCHE=1;
        

    }
    else {
        robotState.isAutoControl = FALSE;
//        LED_ORANGE=1;
//        LED_BLANCHE=0;
        
    }
}

void UartProcessDecodedMessage(int function, int payloadLength, unsigned char * payload) {
    //Fonction prenant en entr�e un octet et servant � reconstituer les trames
    robotState.isfunction = function;
    robotState.ispayloadLength = payloadLength ;
    robotState.ispayload = payload;
    
    switch (function) {
        case attente:
            //function = payloadLength;
            
            break;

//        case SET_ROBOT_STATE:
//
//
//            break;

        case SET_ROBOT_MANUAL_CONTROL:

            SetRobotAutoControlState(payload);
            function = attente;
            
            

            break;

            //            case led:
            //                if (payload[0] == 0x49 && payload[1] == 0x31)
            //                    LED_ORANGE = 1;
            //                if (payload[0] == 0x4F && payload[1] == 0x31)
            //                    LED_ORANGE = 0;
            //                if (payload[0] == 0x49 && payload[1] == 0x32)
            //                    LED_BLANCHE = 1;
            //                if (payload[0] == 0x4F && payload[1] == 0x32)
            //                    LED_BLANCHE = 0;
            //                if (payload[0] == 0x49 && payload[1] == 0x33)
            //                    LED_BLEUE = 1;
            //                if (payload[0] == 0x4F && payload[1] == 0x33)
            //                    LED_BLEUE = 0;
            //                function = attente;
            //                break ;

        default:
            function = attente;
            break;
    }
}
