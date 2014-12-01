#ifndef _FUNKCIJE_H_INCLUDED
#define _FUNKCIJE_H_INCLUDED

#define MOTOR_SKUPLJAC_LEVI		'A'
#define MOTOR_SKUPLJAC_DESNI	'B'
#define MOTOR_SKUPLJAC_MAPE		'C'

#define SKUPLJAC_BRZINA			40
#define SKUPLJAC_MAPE_BRZINA	150

#define SHARP_NORMAL_DISTANCE	 60

#define ALL_DETECTIONS_OFF		3
#define ALL_DETECTIONS_ON		2
#define SHARP_ON				1
#define TASTER_SUDAR_ON			0

void otvoriSkupljac(void);
void zatvoriSkupljac(void);
void pokupiMapu(void);
void skupljacCim(unsigned char,unsigned char);
void writeCoord(void);
void setDetections(int,char);
char getDetections(char,signed char);
void writeTime(void);

#endif