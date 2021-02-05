#include "SaveData.h"
#include <cmath>
#include "PTP.h"
#include <iostream>
#include "Scurve_Jerk.h"

using namespace std;

// ==== Parameter Setting ====
vector<double> InitialPos = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0  };
vector<double> FinalPos = { 0.0 ,0.7854, 0.7854, 0.0, 0.35, 0.0 };
vector<double> PosCmd;
vector<double> VelCmd;
vector<double> AccCmd;
vector<double> JerkCmd;
int EndFlag = 0;


vector<double> Vmax = { 2.4933, 2.4933, 2.4933, 2.4933, 3.4700, 5.8667 }; // Velocity Limitation (rad/s)
vector<double> Amax = { 3.4907, 3.4907, 3.4907, 3.4907, 3.4907, 3.4907 }; // Acceleration Limitation (rad/s^2)
vector<double> Jmax = V_MultiplyScalar(Amax, 10);
vector<double> Smax = V_MultiplyScalar(Jmax, 10);

Scurve_Jerk Scurve;

int main()
{

	//SaveData_CreateFile("Data/Scurve_Minimize_Jerk.txt");
	//SaveData_CreateFile("Data/Scurve_TimeSync.txt");
	//SaveData_CreateFile("Data/Scurve_MultiAxisLimitation.txt");
	SaveData_CreateFile("Data/Scurve_Original.txt");

	// Set Initial Pos and Final Pos
	Scurve.Initial(InitialPos, FinalPos);

	// Set Limitation
	Scurve.SetLimitation(Vmax, Amax, Jmax, Smax);

	// Scurve Type
	 Scurve.MultiAxis_TimeSync();
	//Scurve.MultiAxis_MinimizeJerk();

	while (EndFlag == 0) {
		// Original Scurve
		 PTP_Scurve_Joint(InitialPos, FinalPos, PosCmd, VelCmd, AccCmd, EndFlag);
		
		// Scurve_Jerk
		// Scurve.GetCmd(PosCmd, VelCmd, AccCmd, JerkCmd, EndFlag);

		SaveData_Data(PosCmd, VelCmd, AccCmd);
	}


	SaveData_CloseFile();


	system("Pause");
	return 0;
}