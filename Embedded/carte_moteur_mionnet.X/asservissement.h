/* 
 * File:   SetupPidAsservissement.h
 * Author: TP-EO-5
 *
 * Created on 14 janvier 2022, 10:53
 */

#ifndef ASSERVISSEMENT_H
#define	ASSERVISSEMENT_H

typedef struct _PidCorrector
{
    double Kp;
    double Ki ;
    double Kd;
    double erreurProportionelleMax ;
    double erreurIntegraleMax ;
    double erreurDeriveeMax ;
    double erreurIntegrale ;
    double epsilon_1 ;
    double erreur ;
    
    //For Debug onl y
    double corrP ;
    double corrI ;
    double corrD ;
}PidCorrector ;

void SetupPidAsservissement ( volatile PidCorrector * PidCorr , double Kp, double Ki , double Kd, double proportionelleMax ,double integralMax ,double deriveeMax );
double Correcteur ( volatile PidCorrector *PidCorr , double erreur );

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* SETUPPIDASSERVISSEMENT_H */

