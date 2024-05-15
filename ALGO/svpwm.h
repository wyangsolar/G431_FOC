#ifndef __SVPWM_H
#define __SVPWM_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* defines ------------------------------------------------------------------*/
#define SVPWM_SQRT3_2 	0.86602540378443864676372317075294f   // √3 /2
#define SVPWM_SQRT3 		1.7320508075688772935274463415059f   // √3

typedef struct
{
	float  Ts;			//SVPWM周期 us
	float  Udc;			//母线电压 
}svpwm_t;
extern svpwm_t svpwm;
/* function ------------------------------------------------------------------*/
void svpwm_init(float ts, float udc);

void svpwm_calc(float Valpha, float Vbeta, float *Ta, float *Tb, float *Tc);


#endif
