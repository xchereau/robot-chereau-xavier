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

//consigne 
double consigneX = 0.01;
double consigneTheta = 0.02;
//double consigneM1 = 0.03;
//double consigneM2 = 0.04;

//Measure
double measureX = 0.05;
double measureTheta = 0.06;
//double measureM1 = 0.07;
//double measureM2 = 0.08;

//Error
double ErrorX = 0.09;
double ErrorTheta = 0.1;
//double ErrorM1 = 0.11;
//double ErrorM2 = 0.12; 

//Command
double commandX = 0.13;
double commandTheta = 0.14;
//double commandM1 = 0.15;
//double commandM2 = 0.16;

//KP
double KpX = 0.17;
double KpTheta = 0.18;
//double KpM1 = 0.19;
//double KpM2 = 0.2;

//Correction P
double CoPX = 0.21;
double CoPTheta = 0.22;
//double CoPM1 = 0.23;
//double CopM2 = 024;

//Corr P Max
double CorrPMaxX = 0.25;
double CorrPMaxTheta = 0.26;
//double CorrPMaxM1 = 0.27;
//double CorrPMaxM2 = 0.28;

//KI
double KiMax = 0.29;
double KiTheta = 0.3;
//double KiM1 = 0.31;
//double KiM2 = 0.32;

//Correction I
double CoIX = 0.33;
double CoITheta = 0.34;
//double CoIM1 = 0.35;
//double CoIM2 =0.36;

//Corr I Max
double CorrIMaxX = 0.37;
double CorrIMaxTheta = 0.38;
//double CorrIMaxM1 = 0.39;
//double CorrIMaxM2 = 0.4;

//KD
double KdMax = 0.41;
double KdTheta = 0.42;
//double KdM1 = 0.43;
//double KdM2 = 0.44;

//Correction D
double CoDX = 0.45;
double CoDTheta = 0.46;
//double CoDM1 = 0.47;
//double CoDM2 = 0.48;

//Corr D Max
double CorrDMaxX = 0.49;
double CorrDMaxTheta = 0.5;
//double CorrDMaxM1 = 0.51;
//double CorrDMaxM2 = 0.52;



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
    
    unsigned char positionPayload1 [104] ;
    getBytesFromInt32 ( positionPayload1, 0, consigneX) ;
    getBytesFromFloat ( positionPayload1, 4, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 8, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 12, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 16, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 20, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 24, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 28, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 32, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 36, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 40, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 44, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 48, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 52, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 56, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 60, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 64, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 68, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 72, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 76, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 80, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 84, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 88, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 92, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 96, ( float ) ( consigneX )) ;
    getBytesFromFloat ( positionPayload1, 100, ( float ) ( consigneX )) ;
    
    
    UartEncodeAndSendMessage (POSITION_DATA, 104 , positionPayload1) ;
    
    

}