#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>
#include <libpic30.h>
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "ADC.h"
#include "Robot.h"
#include "main.h"
#include "UART.h"
#include "CB_TX1.h"
#include "CB_RX1.h"
#include "UART_Protocol.h"
#include "automatique.h"
#include "manuelle.h"
#include "Utilities.h"

#define PI 3.1415
#define POSITION_DATA 0x0061
#define FREQ_ECH_QEI 250
#define DISTROUES 0.2182
#define NUMBER_OF_POINTS 8192
#define POINT_TO_METER 0.000016336 //(wheel diameter * PI/ NUMBER_OF_POINTS)

// Point to meter = (dist_roue * pi) / nombre de points codeur


void InitQEI1 ()
{
    QEI1IOCbits.SWPAB = 1 ; //QEAx and QEBx are swapped
    QEI1GECL = 0xFFFF;
    QEI1GECH = 0xFFFF;
    QEI1CONbits.QEIEN = 1 ; // Enable QEI Module
}
void InitQEI2 () 
{
    QEI2IOCbits.SWPAB = 1 ; //QEAx and QEBx are not swapped
    QEI2GECL = 0xFFFF;
    QEI2GECH = 0xFFFF;
    QEI2CONbits.QEIEN = 1 ; // Enable QEI Module
}

double QeiDroitPosition = 0 ;
double QeiGauchePosition = 0  ;

void QEIUpdateData ()
{
//On sauvegarde les anciennes valeurs
    double QeiDroitPosition_T_1 = QeiDroitPosition ;
    double QeiGauchePosition_T_1 = QeiGauchePosition ;

//On réactualise les valeurs des positions
    long QEI1RawValue = POS1CNTL;
    QEI1RawValue += ((long)POS1HLD<<16);
    
    long QEI2RawValue = POS2CNTL;
    QEI2RawValue += ((long)POS2HLD<<16);
    
// Conversion en mm ( r\?egl\?e pour la taille des roue scodeuses )
    QeiDroitPosition = POINT_TO_METER * QEI1RawValue ;
     QeiGauchePosition = -POINT_TO_METER * QEI2RawValue ;
    
// Calcul des deltas de position
    double delta_d = QeiDroitPosition - QeiDroitPosition_T_1 ;
    double delta_g = QeiGauchePosition - QeiGauchePosition_T_1 ;
    
// delta_theta = atan ((delta_d ? delta_g )/ DISTROUES) ;
    double delta_theta = ( delta_d - delta_g ) / DISTROUES;
    double dx = ( delta_d + delta_g ) / 2 ;
    
// Calcul des vitesses
//attention à remultiplier par la fréquence d'échantillonnage
    robotState.vitesseDroitFromOdometry =  delta_d * FREQ_ECH_QEI;
    robotState.vitesseGaucheFromOdometry =  delta_g * FREQ_ECH_QEI;
    robotState.vitesseLineaireFromOdometry = (robotState.vitesseDroitFromOdometry + robotState.vitesseGaucheFromOdometry ) / 2 ;
    robotState.vitesseAngulaireFromOdometry = delta_theta * FREQ_ECH_QEI;
    
//Mise à jour du positionnement terrain à t-1
    robotState.xPosFromOdometry_1 = robotState.xPosFromOdometry ;
    robotState.yPosFromOdometry_1 = robotState.yPosFromOdometry ;
    robotState.angleRadianFromOdometry_1 = robotState.angleRadianFromOdometry ;
    
// Calcul des positions dans le referentiel du terrain
    robotState.xPosFromOdometry = robotState.xPosFromOdometry_1 + robotState.vitesseLineaireFromOdometry / FREQ_ECH_QEI * cos(robotState.angleRadianFromOdometry_1);
    robotState.yPosFromOdometry = robotState.yPosFromOdometry_1 + robotState.vitesseLineaireFromOdometry / FREQ_ECH_QEI * sin(robotState.angleRadianFromOdometry_1);
    robotState.angleRadianFromOdometry = robotState.angleRadianFromOdometry_1 + delta_theta ;
    if ( robotState.angleRadianFromOdometry > PI )
    robotState.angleRadianFromOdometry -= 2*PI ;
    if ( robotState.angleRadianFromOdometry < -PI )
    robotState.angleRadianFromOdometry += 2*PI ;
    
}

void SendPositionData()
{
    
unsigned char positionPayload [24] ;
getBytesFromInt32 ( positionPayload,0 , timestamp ) ;
getBytesFromFloat ( positionPayload , 4 , ( float ) ( robotState. xPosFromOdometry ) ) ;
getBytesFromFloat ( positionPayload ,8 , ( float) ( robotState.yPosFromOdometry ) ) ;
getBytesFromFloat ( positionPayload , 12 , ( float ) ( robotState.angleRadianFromOdometry ) ) ;
getBytesFromFloat ( positionPayload , 16 , ( float ) ( robotState . vitesseLineaireFromOdometry ) ) ;
getBytesFromFloat ( positionPayload , 20 , ( float ) ( robotState.vitesseAngulaireFromOdometry ) ) ;
UartEncodeAndSendMessage (POSITION_DATA, 24 , positionPayload) ;

}