#ifndef UTILITIES_H
#define	UTILITIES_H

#define PI 3.141592653589793

double Abs(double value);
double Max(double value, double value2);
double Min(double value,double value2);
double LimitToInterval(double value, double min, double max);
double Modulo2PIAngleRadian(double angleRadian) ;
double ModuloByAngle(double angleToCenterAround, double angleToCorrect);
float getFloat(unsigned char *p, int index);
double getDouble(unsigned char *p, int index);
void getBytesFromFloat(unsigned char *p, int index, float f);
void getBytesFromInt32(unsigned char *p, int index, long in);
void getBytesFromDouble(unsigned char *p, int index, double d);
float moduloByAngle(double thetawp, double thetaRob );
void changeRef(float *x, float *y, float theta , float xref, float yref);

#endif /*UTILITIES_H*/

