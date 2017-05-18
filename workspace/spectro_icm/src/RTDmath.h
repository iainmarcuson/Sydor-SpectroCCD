/*
 * RTDmath.h
 *
 *  Created on: Aug 6, 2014
 *      Author: freddy
 */

#ifndef RTDMATH_H_
#define RTDMATH_H_

#include <math.h>	// Needs the linkerflag [-lm]

float T_rtd (float r);
float R_rtd (float t);
float Tmin_rtd ();
float Tmax_rtd ();
float Rmin_rtd ();
float Rmax_rtd ();

#endif /* RTDMATH_H_ */
