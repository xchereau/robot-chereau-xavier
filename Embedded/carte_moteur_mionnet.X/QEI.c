#include <xc.h>
#include <math.h>
#include "QEI.h"
#include "Robot.h"
#include "main.h"
#include "IO.h"
#define DISTROUES 281.2
#define PI 3.141592653589793

long QeiDroitPosition_T_1;
long QeiDroitPosition;
long QeiGauchePosition_T_1;
long QeiGauchePosition;
long delta_d;
long delta_g;
long delta_theta;
long  dx;

int FREQ_ECH_QEI=250;

void InitQEI1()
{
QEI1IOCbits.SWPAB = 1 ; //QEAx and QEBx are swapped
QEI1GECL = 0xFFFF;
QEI1GECH = 0xFFFF;
QEI1CONbits.QEIEN = 1 ; // Enable QEI Module
}
void InitQEI2() {
QEI2IOCbits.SWPAB = 1 ; //QEAx and QEBx are not swapped
QEI2GECL = 0xFFFF;
QEI2GECH = 0xFFFF;
QEI2CONbits.QEIEN = 1 ; // Enable QEI Module
}


void QEIUpdateData ()
{
    //On sauvegarde les anciennes valeurs
    QeiDroitPosition_T_1 = QeiDroitPosition ;
    QeiGauchePosition_T_1 = QeiGauchePosition ;
    
    //On r�actualise les valeurs des positions
    long QEI1RawValue = POS1CNTL;
    QEI1RawValue += ( ( long )POS1HLD<<16);
    
    long QEI2RawValue = POS2CNTL;
    QEI2RawValue += ( ( long )POS2HLD<<16);
    
    // Conversion en mm ( r \ ? e g l \ ? e pour la taille des roues codeuses )
    QeiDroitPosition = 0.01620 * QEI1RawValue ;
    QeiGauchePosition = - 0.01620 * QEI2RawValue ;
    
    // Calcul des deltas de position
    delta_d = QeiDroitPosition - QeiDroitPosition_T_1 ;
    delta_g = QeiGauchePosition - QeiGauchePosition_T_1 ;
    
    // delta_ theta = atan (( delta_d ? del ta_g ) / DISTROUES) ;
    delta_theta = ( delta_d - delta_g ) / DISTROUES;
    dx = ( delta_d + delta_g)/2 ;
    
    // Calcul des vitesses
    // attention � remultiplier par la fr�quence d'�chantillonnage
    robotState.vitesseDroitFromOdometry = delta_d * FREQ_ECH_QEI;
    robotState.vitesseGaucheFromOdometry = delta_g * FREQ_ECH_QEI;
    robotState.vitesseLineaireFromOdometry = (robotState.vitesseDroitFromOdometry + robotState.vitesseGaucheFromOdometry ) / 2 ;
    robotState.vitesseAngulaireFromOdometry = delta_theta * FREQ_ECH_QEI ;
    
    
    //Mise � jour du positionnement terrain � t?1
    robotState.xPosFromOdometry_1 = robotState.xPosFromOdometry ;
    robotState.yPosFromOdometry_1 = robotState.yPosFromOdometry ;
    robotState.angleRadianFromOdometry_1 = robotState.angleRadianFromOdometry ;
    
    // Calcul des positions dans le referentiel du terrain
    
    robotState.angleRadianFromOdometry += (robotState.vitesseAngulaireFromOdometry)*1/FREQ_ECH_QEI ;
    if (robotState.angleRadianFromOdometry > PI )
        robotState.angleRadianFromOdometry += -2*PI ;
    if (robotState. angleRadianFromOdometry < -PI )
        robotState.angleRadianFromOdometry += 2*PI ;
    robotState.xPosFromOdometry += robotState.vitesseLineaireFromOdometry * cos(robotState.angleRadianFromOdometry);
    robotState.yPosFromOdometry += robotState.vitesseLineaireFromOdometry * sin(robotState.angleRadianFromOdometry);
}
