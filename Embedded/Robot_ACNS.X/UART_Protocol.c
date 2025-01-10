#include <xc.h>
#include "UART_Protocol.h"
#include "CB_TX1.h"
#include <stdint.h>
#include "timer.h"
#include "IO.h"
#include "CB_RX1.h"
#include "main.h"
#include "Utilities.h"
#include "asservissement.h"
#include "Robot.h"
#include "GhostManager.h"

extern volatile GhostPosition ghostPosition;

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    //Fonction prenant entree la trame et sa longueur pour calculer le checksum
    unsigned char checksum = 0;
    checksum ^= 0xFE;
    checksum ^= (unsigned char) (msgFunction >> 8);
    checksum ^= (unsigned char) msgFunction;
    checksum ^= (unsigned char) (msgPayloadLength >> 8);
    checksum ^= (unsigned char) msgPayloadLength;

    for (int i = 0; i < msgPayloadLength; i++) {
        checksum ^= msgPayload[i];
    }

    return checksum;

}

void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    //Fonction d?encodage et d?envoi d?un message
    unsigned char message[msgPayloadLength + 6];
    int position = 0;
    message[position++] = 0xFE;
    message[position++] = (unsigned char) (msgFunction >> 8);
    message[position++] = (unsigned char) (msgFunction);
    message[position++] = (unsigned char) (msgPayloadLength >> 8);
    message[position++] = (unsigned char) msgPayloadLength;

    for (int i = 0; i < msgPayloadLength; i++) {
        message[position++] = msgPayload[i];
    }

    message[position++] = UartCalculateChecksum(msgFunction, msgPayloadLength, msgPayload);

    SendMessage(message, msgPayloadLength + 6);

}

unsigned char rcvState = STATE_ATTENTE;
int msgDecodedFunction = 0;
int msgDecodedPayloadLength = 0;
unsigned char msgDecodedPayload[128];
int msgDecodedPayloadIndex = 0;
unsigned char calculatedChecksum;
unsigned char receivedChecksum;

void UartDecodeMessage(unsigned char c) {
    //Fonction prenant en entree un octet et servant a reconstituer les trames

    switch (rcvState) {
        case STATE_ATTENTE:
            if (c == 0xFE)
                rcvState = STATE_FUNCTION_MSB;
            break;
        case STATE_FUNCTION_MSB:
            msgDecodedFunction = c << 8;
            rcvState = STATE_FUNCTION_LSB;
            break;
        case STATE_FUNCTION_LSB:
            msgDecodedFunction |= c;
            rcvState = STATE_PAYLOAD_LENGTH_MSB;
            break;
        case STATE_PAYLOAD_LENGTH_MSB:
            msgDecodedPayloadLength = c << 8;
            rcvState = STATE_PAYLOAD_LENGTH_LSB;
            break;
        case STATE_PAYLOAD_LENGTH_LSB:
            msgDecodedPayloadLength |= c;
            if (msgDecodedPayloadLength < 1024) {
                if (msgDecodedPayloadLength > 0) {
                    rcvState = STATE_PAYLOAD;
                } else {
                    rcvState = STATE_CHECKSUM;
                }
            } else {
                rcvState = STATE_ATTENTE;
            }
            break;
        case STATE_PAYLOAD:
            if (msgDecodedPayloadIndex <= msgDecodedPayloadLength) {
                msgDecodedPayload[msgDecodedPayloadIndex] = c;
                if (++msgDecodedPayloadIndex >= msgDecodedPayloadLength) {
                    rcvState = STATE_CHECKSUM;
                    msgDecodedPayloadIndex = 0;
                }

            }
            break;
        case STATE_CHECKSUM:
            calculatedChecksum = c;

            receivedChecksum = UartCalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            if (calculatedChecksum == receivedChecksum) {
                //Success, on a un message valide
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            } else {
                //print("Les checksums sont différents");
            }
            rcvState = STATE_ATTENTE;
            break;
        default:
            rcvState = STATE_ATTENTE;
            break;
    }
}


void UartProcessDecodedMessage(int rcvFunction, int payloadLength, unsigned char* payload) {
    //Fonction appelee apres le decodage pour executer l?action correspondant au message recu
    switch (rcvFunction) {
        case FUNCTION_TEXT:
            UartEncodeAndSendMessage(0x0080, payloadLength, payload);
            break;
        case FUNCTION_LED1:
            LED_BLANCHE_1 = payload[0];
            break;
        case FUNCTION_LED2:
            LED_BLEUE_1 = payload[0];
            break;
        case FUNCTION_LED3:
            LED_ORANGE_1 = payload[0];
            break;
        case SET_ROBOT_STATE:
            SetRobotState(payload[0]);
            break;
        
        case CONFIG_PID:
            SetupPidAsservissement(&robotState.PidX, getFloat(payload, 0),  getFloat(payload, 4),  
                     getFloat(payload, 8), getFloat(payload, 12), getFloat(payload, 16), getFloat(payload, 20));
            SetupPidAsservissement(&robotState.PidTheta, getFloat(payload, 24),  getFloat(payload, 28),  
                     getFloat(payload, 32), getFloat(payload, 36), getFloat(payload, 40), getFloat(payload, 44));
            break;
            
        case CONFIG_VLINEAIRE:
            robotState.consigneVitesseLineaire = getFloat(payload, 0);
            break;
            
        case CONFIG_VANGULAIRE:
            robotState.consigneVitesseAngulaire = getFloat(payload, 0);
            break;
        
        case SET_GHOST_POSITION:
            ghostPosition.targetX = getFloat(payload, 0);
            ghostPosition.targetY = getFloat(payload, 4);
            break;
            
        default:
            break;

    }

}

//*************************************************************************/
//Fonctions correspondant aux messages
//*************************************************************************/

void robotStateChange(unsigned char rbState ) {
    unsigned char msg[5];
    int position = 0;
    unsigned long tstamp = timestamp;
    msg[position++] = rbState;
    msg[position++] = (unsigned char) (tstamp >> 24);
    msg[position++] = (unsigned char) (tstamp >> 16);
    msg[position++] = (unsigned char) (tstamp >> 8);
    msg[position++] = (unsigned char) tstamp;

    UartEncodeAndSendMessage(0x0050, 5, msg);
}

#include "GhostManager.h"

extern Waypoint_t waypoints[MAX_POS];
extern volatile GhostPosition ghostPosition;
extern int waypoint_index;
volatile int waypoint_received;
extern volatile int mooving;

unsigned char rcvState_UART2 = STATE_ATTENTE;
int msgDecodedFunction_UART2 = 0;
int msgDecodedPayloadLength_UART2 = 0;
unsigned char msgDecodedPayload_UART2[128];
int msgDecodedPayloadIndex_UART2 = 0;
unsigned char calculatedChecksum_UART2;
unsigned char receivedChecksum_UART2;

int x = 0;
int y = 0;
int sign = 0;
float tempX , tempY;

void UartProcessDecodedMessage_UART2(int rcvFunction, int payloadLength, unsigned char* payload) {
    //Fonction appelee apres le decodage pour executer l?action correspondant au message recu
    switch (rcvFunction) {
        case 0x98:
            x = payload[3] | payload[2] << 8 | payload[1] << 16 | payload[0] << 24;
            y = payload[7] | payload[6] << 8 | payload[5] << 16 | payload[4] << 24;
            
            tempX = ((float) x) / 1000.0f + 0.03f;
            tempY = ((float) y) / 1000.0f;
            
            changeRef(&tempX,&tempY,ghostPosition.theta, ghostPosition.x, ghostPosition.y);
            
            Waypoint_t nWaypoint = {
               tempX,
               tempY,
                0
            };
            
            //if(waypoint_index != MAX_POS) {
                waypoints[0] = nWaypoint;
            //}
            break;
            
        case 0x99:
            x = payload[3] | payload[2] << 8 | payload[1] << 16 | payload[0] << 24;
            y = payload[7] | payload[6] << 8 | payload[5] << 16 | payload[4] << 24;
            
            
            
            tempX = ((float) x) / 1000.0f + 0.03f;
            tempY = -((float) y) / 1000.0f;
            
            changeRef(&tempX,&tempY,ghostPosition.theta, ghostPosition.x, ghostPosition.y);
            nWaypoint.x = tempX;
            nWaypoint.y = tempY;
            nWaypoint.last_rotate = 0;
            
            //if(waypoint_index != MAX_POS) {
                waypoints[0] = nWaypoint;
            //}
            break;        
        default:
            break;

    }
}
void UartDecodeMessage_UART2(unsigned char c) {
    //Fonction prenant en entree un octet et servant a reconstituer les trames

    switch (rcvState_UART2) {
        case STATE_ATTENTE:
            if (c == 0xFE)
                rcvState_UART2 = STATE_FUNCTION_MSB;
            break;
        case STATE_FUNCTION_MSB:
            msgDecodedFunction_UART2 = c << 8;
            rcvState_UART2 = STATE_FUNCTION_LSB;
            break;
        case STATE_FUNCTION_LSB:
            msgDecodedFunction_UART2 |= c;
            rcvState_UART2 = STATE_PAYLOAD_LENGTH_MSB;
            break;
        case STATE_PAYLOAD_LENGTH_MSB:
            msgDecodedPayloadLength_UART2 = c << 8;
            rcvState_UART2 = STATE_PAYLOAD_LENGTH_LSB;
            break;
        case STATE_PAYLOAD_LENGTH_LSB:
            msgDecodedPayloadLength_UART2 |= c;
            if (msgDecodedPayloadLength_UART2 < 1024) {
                if (msgDecodedPayloadLength_UART2 > 0) {
                    rcvState_UART2 = STATE_PAYLOAD;
                } else {
                    rcvState_UART2 = STATE_CHECKSUM;
                }
            } else {
                rcvState_UART2 = STATE_ATTENTE;
            }
            break;
        case STATE_PAYLOAD:
            if (msgDecodedPayloadIndex_UART2 <= msgDecodedPayloadLength_UART2) {
                msgDecodedPayload_UART2[msgDecodedPayloadIndex_UART2] = c;
                if (++msgDecodedPayloadIndex_UART2 >= msgDecodedPayloadLength_UART2) {
                    rcvState_UART2 = STATE_CHECKSUM;
                    msgDecodedPayloadIndex_UART2 = 0;
                }

            }
            break;
        case STATE_CHECKSUM:
            calculatedChecksum_UART2 = c;

            receivedChecksum_UART2 = UartCalculateChecksum(msgDecodedFunction_UART2, msgDecodedPayloadLength_UART2, msgDecodedPayload_UART2);
            if (calculatedChecksum_UART2 == receivedChecksum_UART2) {
                //Success, on a un message valide
                if (mooving == 0){
                    InitTrajectoryGenerator();
                    UartProcessDecodedMessage_UART2(msgDecodedFunction_UART2, msgDecodedPayloadLength_UART2, msgDecodedPayload_UART2);
                    waypoint_received = 1;
                }
            } else {
               // UartProcessDecodedMessage_UART2(msgDecodedFunction_UART2, msgDecodedPayloadLength_UART2, msgDecodedPayload_UART2);
            }
            rcvState_UART2 = STATE_ATTENTE;
            break;
        default:
            rcvState_UART2 = STATE_ATTENTE;
            break;
    }
}




