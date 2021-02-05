#include "PTP.h"


//================ 點對點命令 ================

double Time = 0.0;
vector<double> jerk = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
vector<double> Displacement;

double DisplacementFunction;
double max_time = 0.0;
double Ta, Tb, Tc, Ts, t1, t2, t3, t4, t5, t6, t7;
double ta, tb, tc, ts;
int NumberMax;


void PTP_Scurve_Joint(vector<double> InitialPos, vector<double> FinalPos, vector<double>(&PosCmd), vector<double>(&VelCmd), vector<double>(&AccCmd), int(&EndFlag))
{
	vector<double> Vmax = { 2.4933, 2.4933, 2.4933, 2.4933, 3.4700, 5.8667 }; // Velocity Limitation (rad/s)
	vector<double> Amax = { 3.4907, 3.4907, 3.4907, 3.4907, 3.4907, 3.4907 }; // Acceleration Limitation (rad/s^2)

	vector<double> Aavg = V_MultiplyScalar(Amax, 0.75); // Average Accleration (0.75 * Max Acceleration)

	if (Time == 0)
	{
		Displacement = V_SubVector(FinalPos, InitialPos);

		for (size_t i = 0; i < AXIS; i++)
		{
			
			ta = Vmax[i] / Aavg[i];
			tb = 2 * Vmax[i] / Amax[i] - ta;
			tc = (ta - tb) / 2;
			ts = (Displacement[i] - Vmax[i] * ta) / Vmax[i];

			if (ts < 0)
			{
				ts = 0;
				Vmax[i] = sqrt(abs(Displacement[i] * Aavg[i]));
			}

			if ((ts + 2 * ta) > max_time)
			{
				Tc = tc;
				Ta = ta;
				Ts = ts;
				Tb = tb;

				max_time = ts + 2 * ta;

			}

			if (Displacement[i] < 0)
			{
				Amax[i] = -Amax[i];
				Vmax[i] = -Vmax[i];
				Aavg[i] = -Aavg[i];
			}


		}
		t1 = Tc;
		t2 = Tc + Tb;
		t3 = Ta;
		t4 = Ta + Ts;
		t5 = Ta + Ts + Tc;
		t6 = Ta + Ts + Tc + Tb;
		t7 = Ta + Ts + Ta;

		//================ DisplacementFunction ================
		DisplacementFunction =
			pow(t1, 3) / 6 + pow(t2, 3) / 6 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6 - pow(t6, 3) / 6 + pow(t7, 3) / 6
			- t1 * pow(t3, 2) / 2 - pow(t1, 2)*t7 / 2 + t1*t3*t7
			- t2 * pow(t3, 2) / 2 - pow(t2, 2)*t7 / 2 + t2*t3*t7
			- pow(t3, 2)*t7 / 2 - pow(t4, 2) * t7 / 2 + t4*pow(t7, 2) / 2
			+ pow(t5, 2)*t7 / 2 - t5 * pow(t7, 2) / 2
			+ pow(t6, 2)*t7 / 2 - t6 * pow(t7, 2) / 2;

		//================ Calculate Jerk ================

		jerk = V_MultiplyScalar(Displacement, 1 / DisplacementFunction);

	}

	if (Time <= t7)
	{
		for (int j = 0; j < 1; j++)
		{
			if (t1 >= Time)
			{

				AccCmd = V_MultiplyScalar(jerk , Time);
				VelCmd = V_MultiplyScalar(jerk, (pow(Time, 2) / 2));
				PosCmd = V_AddVector(InitialPos , V_MultiplyScalar(jerk, (pow(Time, 3) / 6)));

			}
			else if (t2 >= Time && Time > t1)
			{

				AccCmd = V_MultiplyScalar(jerk , t1);
				VelCmd = V_MultiplyScalar(jerk , (-pow(t1, 2) / 2 + t1*Time));
				PosCmd = V_AddVector(InitialPos , V_MultiplyScalar(jerk, (pow(t1, 3) / 6 ) - Time * pow(t1, 2) / 2 + t1 * pow(Time, 2) / 2));

			}
			else if (t3 >= Time && Time > t2)
			{

				AccCmd = V_MultiplyScalar(jerk, (t1 + t2 - Time));
				VelCmd = V_MultiplyScalar(jerk, (-pow(t1, 2) / 2 - pow(t2, 2) / 2 + Time * (t2 + t1) - pow(Time, 2) / 2));
				PosCmd = V_AddVector(InitialPos, V_MultiplyScalar(jerk, (pow(t1, 3) / 6 + pow(t2, 3) / 6 - Time * (pow(t1, 2) / 2 + pow(t2, 2) / 2) + pow(Time, 2) * (t2 + t1) / 2 - pow(Time, 3) / 6)));

			}
			else if (t4 >= Time && Time > t3)
			{

				AccCmd = V_MultiplyScalar(jerk, 0);
				VelCmd = V_MultiplyScalar(jerk, (-pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 + t1 * t3 + t2*t3));
				PosCmd = V_AddVector(InitialPos, V_MultiplyScalar(jerk, (pow(t1, 3) / 6 + pow(t2, 3) / 6 + pow(t3, 3) / 3 - t1 * pow(t3, 2) / 2 - t2 * pow(t3, 2) / 2 + Time * (-pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 + t1 * t3 + t2 * t3))));

			}
			else if (t5 >= Time && Time > t4)
			{

				AccCmd = V_MultiplyScalar(jerk, (t4 - Time));
				VelCmd = V_MultiplyScalar(jerk, (-pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + t1 * t3 + t2 * t3 + Time * t4 - pow(Time, 2) / 2));
				PosCmd = V_AddVector(InitialPos, V_MultiplyScalar(jerk, (pow(t1, 3) / 6 + pow(t2, 3) / 6 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - t1 * pow(t3, 2) / 2 - t2 * pow(t3, 2) / 2 + Time * (-pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + t1 * t3 + t2 * t3) + t4 * pow(Time, 2) / 2 - pow(Time, 3) / 6)));

			}
			else if (t6 >= Time && Time > t5)
			{

				AccCmd = V_MultiplyScalar(jerk, (t4 - t5));
				VelCmd = V_MultiplyScalar(jerk, (-pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + pow(t5, 2) / 2 + t1 * t3 + t2 * t3 - Time * (t5 - t4)));
				PosCmd = V_AddVector(InitialPos, V_MultiplyScalar(jerk, (pow(t1, 3) / 6 + pow(t2, 3) / 6 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6 - t1 * pow(t3, 2) / 2 - t2 * pow(t3, 2) / 2 + Time * (-pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + pow(t5, 2) / 2 + t1 * t3 + t2 * t3) - pow(Time, 2) * (t5 - t4) / 2)));

			}
			else if (t7 >= Time && Time > t6)
			{

				AccCmd = V_MultiplyScalar(jerk, (Time - t6 - t5 + t4));
				VelCmd = V_MultiplyScalar(jerk, (-pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + pow(t5, 2) / 2 + pow(t6, 2) / 2 + t1 * t3 + t2 * t3 + Time * (-t6 - t5 + t4) + pow(Time, 2) / 2));
				PosCmd = V_AddVector(InitialPos, V_MultiplyScalar(jerk, (pow(t1, 3) / 6 + pow(t2, 3) / 6 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6 - t1 * pow(t3, 2) / 2 - t2 * pow(t3, 2) / 2 - pow(t6, 3) / 6 + Time * (-pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + pow(t5, 2) / 2 + pow(t6, 2) / 2 + t1 * t3 + t2 * t3) - pow(Time, 2) * (t5 - t4 + t6) / 2 + pow(Time, 3) / 6)));

			}
		}
		Time = Time + SamplingTime;

		EndFlag = 0;  // 判別PTP命令是否結束(未結束)
	}
	else
	{
		Time = 0;
		max_time = 0;

		PosCmd = FinalPos;
		VelCmd = V_MultiplyScalar(jerk, 0);
		AccCmd = V_MultiplyScalar(jerk, 0);


		EndFlag = 1; // 判別PTP命令是否結束(結束)

	}




}
