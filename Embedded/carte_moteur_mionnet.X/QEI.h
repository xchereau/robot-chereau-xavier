
#ifndef QEI_H
#define	QEI_H

#ifdef	__cplusplus
extern "C" {
#endif

void InitQEI2 (void) ;
void InitQEI1 (void);
void QEIUpdateData (void);
void SendPositionData(void);

#ifdef	__cplusplus
}
#endif

#endif	/* QEI_H */

