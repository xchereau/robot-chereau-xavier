#ifndef MANUELLE_H
#define	MANUELLE_H



void commande_manuelle (int function, int payloadLength, unsigned char * payload);
#define attente 0
#define SET_ROBOT_STATE 1
#define led 2
void manuelle(int function, int payloadLength, unsigned char * payload);

#endif

