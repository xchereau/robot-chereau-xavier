
#ifndef QEI_H
#define	QEI_H

#ifdef	__cplusplus
extern "C" {
#endif

void InitQEI2 (void) ;
void InitQEI1 (void);
void QEIUpdateData (void);
void SendPositionData(void);

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

#ifdef	__cplusplus
}
#endif

#endif	/* QEI_H */

