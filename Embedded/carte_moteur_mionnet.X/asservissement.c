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
#include "asservissement.h"
#include "QEI.h"

#define FREQ_ECH_QEI 250

SetupPidAsservissement ( volatile PidCorrector * PidCorr , double Kp, double Ki , double Kd, double proportionelleMax , double integralMax ,double deriveeMax )
{
PidCorr->Kp = Kp;
PidCorr->erreurProportionelleMax = proportionelleMax ; //On limite la correction due au Kp
PidCorr->Ki = Ki ;
PidCorr->erreurIntegraleMax = integralMax ; //On limite la correction due au Ki
PidCorr->Kd = Kd;
PidCorr->erreurDeriveeMax = deriveeMax ;
}

double Correcteur ( volatile PidCorrector *PidCorr , double erreur )
{
 PidCorr->erreur = erreur;
    double erreurProportionnelle = LimitToInterval(erreur, -PidCorr->erreurProportionelleMax/PidCorr->Kp, PidCorr->erreurProportionelleMax/PidCorr->Kp);
    PidCorr->corrP = erreurProportionnelle * PidCorr->Kp;
    
    PidCorr->erreurIntegrale += erreur /FREQ_ECH_QEI;
    PidCorr->erreurIntegrale = LimitToInterval(PidCorr->erreurIntegrale, -PidCorr->erreurIntegraleMax/PidCorr->Ki, PidCorr->erreurIntegraleMax/PidCorr->Ki);
    PidCorr->corrI = PidCorr->erreurIntegrale * PidCorr->Ki;
    
    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
    double deriveeBornee = LimitToInterval(erreurDerivee, -PidCorr->erreurDeriveeMax/PidCorr->Kd, PidCorr->erreurDeriveeMax/PidCorr->Kd);
    PidCorr->epsilon_1 = erreur;
    PidCorr->corrD = PidCorr->Kd * deriveeBornee;
    
    return PidCorr->corrP + PidCorr->corrI + PidCorr->corrD;
}


