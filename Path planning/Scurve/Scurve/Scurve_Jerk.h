#pragma once
#include "Setting.h"
#include "vector_tool.h"
class Scurve_Jerk
{
private:
	vector<double> InitialPos;
	vector<double> FinalPos;
	double a = sqrt(3)/2;
	int number = 0;

public:
	// Parameter
	vector<double> Distance;
	vector<double> Vmax; vector<double> Amax; vector<double> Jmax; vector<double> Smax;
	vector<double> Ts;	vector<double> Tj; vector<double> Tv; vector<double> Ta; vector<double> Total_Time;
	vector<vector<double>> PCmd; vector<vector<double>> VCmd; vector<vector<double>> ACmd; vector<vector<double>> JCmd;
	int Endflag; int endelement;
	
	// Function 
	void Initial(vector<double> InitialPos, vector<double> FinalPos);
	void SetLimitation(vector<double>V, vector<double>A, vector<double>J, vector<double>S);
	void MultiAxis_TimeSync();
	void MultiAxis_MinimizeJerk();
	void GetCmd(vector<double>&PosCmd, vector<double> &VelCmd, vector<double> &AccCmd, vector<double> &JerkCmd, int &Endflag);
};

