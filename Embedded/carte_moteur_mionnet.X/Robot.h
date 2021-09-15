#ifndef ROBOT_H
#define ROBOT_H

typedef enum {
    FALSE = 0,
    TRUE = 1,
} boolean;
extern volatile boolean mybool;

typedef struct robotStateBITS {
    union {

        struct 
        {
            unsigned char taskEnCours;
          
            float vitesseGaucheConsigne;
            float vitesseGaucheCommandeCourante;
            float vitesseDroiteConsigne;
            float vitesseDroiteCommandeCourante;
            float distanceTelemetreExtremeDroit;
            float distanceTelemetreDroit;
            float distanceTelemetreCentre;
            float distanceTelemetreGauche;
            float distanceTelemetreExtremeGauche;
            float moteur_gauche; 
            float moteur_droit;
            
            boolean isAutoControl;
            float isfunction;
            float ispayloadLength;
            unsigned char* ispayload;
            
            double vitesseDroitFromOdometry;
            double vitesseGaucheFromOdometry;
            double vitesseLineaireFromOdometry;
            double vitesseAngulaireFromOdometry;
            double xPosFromOdometry_1;
            double xPosFromOdometry;
            double yPosFromOdometry_1;
            double yPosFromOdometry ;
            double angleRadianFromOdometry_1;
            double angleRadianFromOdometry;
            
            
        }
    ;}
;} 
ROBOT_STATE_BITS;

extern volatile ROBOT_STATE_BITS robotState;

#endif