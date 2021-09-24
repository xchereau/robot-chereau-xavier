#ifndef MANUELLE_H
#define	MANUELLE_H



void commande_manuelle (int function, int payloadLength, unsigned char * payload);
#define attente 0
#define SET_ROBOT_STATE 0x0051
#define led 0x0020
void manuelle(int function, int payloadLength, unsigned char * payload);

#endif

