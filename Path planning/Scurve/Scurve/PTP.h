#ifndef PTP
#define PTP

#include <stdio.h>
#include <math.h>
#include "Setting.h"
#include "vector_tool.h"


void PTP_Scurve_Joint(vector<double> InitialPos, vector<double> FinalPos, vector<double>(&PosCmd), vector<double>(&VelCmd), vector<double>(&AccCmd), int(&EndFlag));


#endif