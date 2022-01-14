#include "robot.h"
#include "main.h"

volatile ROBOT_STATE_BITS robotState ;

typedef struct _PidX
{
    double KpX;
    double KiX ;
    double KdX;
    double erreurProportionelleX ;
    double erreurIntegraleX ;
    double erreurDeriveeX ;
    
    //For Debug onl y
    double corrPX ;
    double corrIX ;
    double corrDX ;
}PidX ;

typedef struct _PidTheta
{
    double KpTheta;
    double KiTheta ;
    double KdTheta;
    double erreurProportionelleTheta ;
    double erreurIntegraleTheta ;
    double erreurDeriveeTheta ;

    //For Debug onl y
    double corrPTheta ;
    double corrITheta ;
    double corrDTheta ;
}PidTheta ;
