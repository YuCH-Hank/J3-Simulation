#include "Scurve_Jerk.h"

// Calculate Limitation =====================================================================================================
void TimeSync(vector<double> Total_Time, double Time_Sync, vector<double> &Ts, vector<double> &Tj, vector<double> &Ta, vector<double> &Tv, double error,
	vector<double> Distance, vector<double> Vmax, vector<double>  Amax, vector<double> &Jmax, vector<double> Smax);
void GenerateCmd(double Ts, double Tj, double Ta, double Tv, double Jmax, double a, vector<double> &Jcmd, vector<double> &Acmd, vector<double> &Vcmd, vector<double> &Pcmd, double InitialPos, double Distance);
vector<double> Get_Time(double Ts, double Tj, double Ta, double Tv);
vector<double> Get_Jerk(vector<double> Time, double Jmax, double a);
vector<double> Integral(vector<double> k);
double Calculate_tau(double t, int a, int b, vector<double>Time);
void Calculate_Tv(double Distance, double Vmax, double Ts, double Tj, double Ta, double &Tv);
void Calculate_Ta(double Distance, double Amax, double Vmax, double Ts, double Tj, double &Ta, double &Tv);
void Calculate_Tj(double Distance, double Jmax, double Amax, double Vmax, double Ts, double &Tj, double &Ta, double &Tv);
void SingleAxis(double Distance, double Smax, double &Jmax, double Amax, double Vmax, double &Ts, double &Tj, double &Ta, double &Tv);
void InitialVector(vector<vector<double>> &PosCmd, vector<vector<double>> &VecCmd, vector<vector<double>> &AccCmd, vector<vector<double>> &JerkCmd, int Dimension);

void Scurve_Jerk::Initial(vector<double> Initial, vector<double> Final)
{
	InitialPos = Initial;
	FinalPos = Final;
	Distance = V_SubVector(Final, Initial);

	for (size_t i = 0; i < Distance.size(); i++)
	{
		Ts.push_back(0);
		Tj.push_back(0);
		Ta.push_back(0);
		Tv.push_back(0);
	}
}

void Scurve_Jerk::SetLimitation(vector<double> V, vector<double> A, vector<double> J, vector<double> S)
{
	Vmax = V;
	Amax = A;
	Jmax = J;
	Smax = S;

}

void Scurve_Jerk::MultiAxis_TimeSync()
{
	// ====================== Calculate Limitation of Each Limitation
	for (size_t i = 0; i < Distance.size(); i++)
	{
		SingleAxis(Distance[i], Smax[i], Jmax[i], Amax[i], Vmax[i], Ts[i], Tj[i], Ta[i], Tv[i]);
	}

	// ====================== Calculate Total Time
	Total_Time = V_AddVector(V_MultiplyScalar(Ts, 8), V_MultiplyScalar(Tj, 4), V_MultiplyScalar(Ta, 2), Tv);

	vector<double> Scaling = V_ScalarDividebyVector(Total_Time, V_max_element(Total_Time));

	Ts = V_MultiplyVector(Ts, Scaling);
	Tj = V_MultiplyVector(Tj, Scaling);
	Ta = V_MultiplyVector(Ta, Scaling);
	Tv = V_MultiplyVector(Tv, Scaling);
	Jmax = V_DivideVector(Jmax, V_Power(Scaling, 3));
	Jmax = V_MultiplyVector(Jmax, V_Sign(Distance));

	vector<double> Jerk(int(V_max_element(Total_Time) / SamplingTime + 1));
	vector<double> Acc(int(V_max_element(Total_Time) / SamplingTime + 1));
	vector<double> Vec(int(V_max_element(Total_Time) / SamplingTime + 1));
	vector<double> Pos(int(V_max_element(Total_Time) / SamplingTime + 1));

	InitialVector(PCmd, VCmd, ACmd, JCmd, Distance.size());

	// ====================== Generate Cmd
	for (int i = 0; i < Distance.size(); i++)
	{

		GenerateCmd(Ts[i], Tj[i], Ta[i], Tv[i], Jmax[i], a, Jerk, Acc, Vec, Pos, InitialPos[i], Distance[i]);
		for (size_t j = 0; j < Jerk.size(); j++)
		{
			if (Distance[i] != 0)
			{
				JCmd[i].push_back(Jerk[j]);
				ACmd[i].push_back(Acc[j]);
				VCmd[i].push_back(Vec[j]);
				PCmd[i].push_back(Pos[j]);
			}
			else
			{
				JCmd[i].push_back(0);
				ACmd[i].push_back(0);
				VCmd[i].push_back(0);
				PCmd[i].push_back(InitialPos[i]);
			}

		}
	}
	endelement = JCmd[0].size();

}

void Scurve_Jerk::MultiAxis_MinimizeJerk()
{
	// ====================== Calculate Limitation of Each Limitation
	for (size_t i = 0; i < Distance.size(); i++)
	{
		SingleAxis(Distance[i], Smax[i], Jmax[i], Amax[i], Vmax[i], Ts[i], Tj[i], Ta[i], Tv[i]);
	}

	// ====================== Calculate Total Time
	Total_Time = V_AddVector(V_MultiplyScalar(Ts, 8), V_MultiplyScalar(Tj, 4), V_MultiplyScalar(Ta, 2), Tv);

	// ====================== Min Jerk Synchronization
	double error = 0.001;
	TimeSync(Total_Time, V_max_element(Total_Time), Ts, Tj, Ta, Tv, error, Distance,Vmax, Amax, Jmax, Smax);
	Jmax = V_MultiplyVector(Jmax, V_Sign(Distance));

	vector<double> Jerk(int(V_max_element(Total_Time) / SamplingTime + 1));
	vector<double> Acc(int(V_max_element(Total_Time) / SamplingTime + 1));
	vector<double> Vec(int(V_max_element(Total_Time) / SamplingTime + 1));
	vector<double> Pos(int(V_max_element(Total_Time) / SamplingTime + 1));
	
	InitialVector(PCmd, VCmd, ACmd, JCmd, Distance.size());

	// ====================== Generate Cmd
	for (int i = 0; i < Distance.size(); i++)
	{

		GenerateCmd(Ts[i], Tj[i], Ta[i], Tv[i], Jmax[i], a, Jerk, Acc, Vec, Pos, InitialPos[i], Distance[i]);
		for (size_t j = 0; j < Jerk.size(); j++)
		{
			if (Distance[i] != 0)
			{
				JCmd[i].push_back(Jerk[j]);
				ACmd[i].push_back(Acc[j]);
				VCmd[i].push_back(Vec[j]);
				PCmd[i].push_back(Pos[j]);
			}
			else
			{
				JCmd[i].push_back(0);
				ACmd[i].push_back(0);
				VCmd[i].push_back(0);
				PCmd[i].push_back(InitialPos[i]);
			}

		}
	}
	vector<double> Element;
	for (size_t i = 0; i < Distance.size(); i++) {
		Element.push_back(JCmd[i].size());
	}
	endelement = *min_element(Element.begin(), Element.end());
}

void Scurve_Jerk::GetCmd(vector<double>&PosCmd, vector<double> &VelCmd, vector<double> &AccCmd, vector<double> &JerkCmd, int &Endflag)
{
	for (size_t i = 0; i < Distance.size(); i++)
	{
		if (number == 0)
		{
			PosCmd.push_back(PCmd[i][number]);
			VelCmd.push_back(VCmd[i][number]);
			AccCmd.push_back(ACmd[i][number]);
			JerkCmd.push_back(JCmd[i][number]);
		}
		else
		{
			PosCmd[i] = PCmd[i][number];
			VelCmd[i] = VCmd[i][number];
			AccCmd[i] = ACmd[i][number];
			JerkCmd[i] = JCmd[i][number];
		}
	}
	number++;
	if (number == endelement)
		Endflag = 1;
	else
		Endflag = 0;
}

// Self Fucntion =======================================================================================
void TimeSync(vector<double> Total_Time, double Time_Sync, vector<double> &Ts, vector<double> &Tj, vector<double> &Ta, vector<double> &Tv, double error,
	vector<double> Distance, vector<double> Vmax, vector<double>  Amax, vector<double> &Jmax, vector<double> Smax)
{
	for (size_t k = 0; k < Distance.size(); k++)
	{
		if (Total_Time[k] == Time_Sync || Distance[k] == 0)
		{
			Jmax[k] = Jmax[k];
			Ts[k] = Ts[k];          Tj[k] = Tj[k];
			Ta[k] = Ta[k];          Tv[k] = Tv[k];
		}
		else
		{ 
			double J_LB = 0.0; double J_UB = Jmax[k];
			double T_UB = Total_Time[k];  double T_LB = 0.0;

			while (abs(Total_Time[k] - Time_Sync) > error)
			{
				Jmax[k] = (J_LB + J_UB) / 2.0;
				SingleAxis(Distance[k], Smax[k], Jmax[k], Amax[k], Vmax[k], Ts[k], Tj[k], Ta[k], Tv[k]);

				Total_Time[k] = 8.0 * Ts[k] + 4.0 * Tj[k] + 2.0  * Ta[k] + 1.0  * Tv[k];

				if (J_LB != 0) {
					if (Total_Time[k] < Time_Sync)
						Jmax[k] = (T_LB * Jmax[k] - Total_Time[k] * J_LB + (J_LB - Jmax[k]) * Time_Sync) / (T_LB - Total_Time[k]);
					else
						Jmax[k] = (Total_Time[k] * J_UB - T_UB * Jmax[k] + (Jmax[k] - J_UB) * Time_Sync) / (Total_Time[k] - T_UB);

					SingleAxis(Distance[k], Smax[k], Jmax[k], Amax[k], Vmax[k], Ts[k], Tj[k], Ta[k], Tv[k]);

					Total_Time[k] = 8.0 * Ts[k] + 4.0 * Tj[k] + 2.0  * Ta[k] + 1.0  * Tv[k];
				}

				if (Total_Time[k] > Time_Sync)
				{
					J_LB = Jmax[k]; T_LB = Total_Time[k];
				}
				else
				{
					J_UB = Jmax[k]; T_UB = Total_Time[k];
				}
			}
			Jmax[k] = Jmax[k];
			Ts[k] = Ts[k];          Tj[k] = Tj[k];
			Ta[k] = Ta[k];          Tv[k] = Tv[k];
		}
	}
}

void InitialVector(vector<vector<double>> &PosCmd, vector<vector<double>> &VecCmd, vector<vector<double>> &AccCmd, vector<vector<double>> &JerkCmd, int Dimension)
{
	vector<double> a;
	for (size_t i = 0; i < Dimension; i++)
	{
		PosCmd.push_back(a);
		VecCmd.push_back(a);
		AccCmd.push_back(a);
		JerkCmd.push_back(a);
	}
}

void GenerateCmd(double Ts, double Tj, double Ta, double Tv, double Jmax, double a, vector<double> &Jcmd, vector<double> &Acmd, vector<double> &Vcmd, vector<double> &Pcmd, double InitialPos, double Distance)
{
	if (Distance != 0)
	{
		vector<double> time = Get_Time(Ts, Tj, Ta, Tv);
		Jcmd = Get_Jerk(time, Jmax, a);
		Acmd = Integral(Jcmd);
		Vcmd = Integral(Acmd);
		Pcmd = V_AddScalar(Integral(Vcmd), InitialPos);
	}

}

vector<double> Get_Time(double Ts, double Tj, double Ta, double Tv)
{
	vector<double> time;
	time.push_back(0.0);
	time.push_back(1 * Ts);
	time.push_back(1 * Ts + 1 * Tj);
	time.push_back(2 * Ts + 1 * Tj);
	time.push_back(2 * Ts + 1 * Tj + 1 * Ta);
	time.push_back(3 * Ts + 1 * Tj + 1 * Ta);
	time.push_back(3 * Ts + 2 * Tj + 1 * Ta);
	time.push_back(4 * Ts + 2 * Tj + 1 * Ta);
	time.push_back(4 * Ts + 2 * Tj + 1 * Ta + 1 * Tv);
	time.push_back(5 * Ts + 2 * Tj + 1 * Ta + 1 * Tv);
	time.push_back(5 * Ts + 3 * Tj + 1 * Ta + 1 * Tv);
	time.push_back(6 * Ts + 3 * Tj + 1 * Ta + 1 * Tv);
	time.push_back(6 * Ts + 3 * Tj + 2 * Ta + 1 * Tv);
	time.push_back(7 * Ts + 3 * Tj + 2 * Ta + 1 * Tv);
	time.push_back(7 * Ts + 4 * Tj + 2 * Ta + 1 * Tv);
	time.push_back(8 * Ts + 4 * Tj + 2 * Ta + 1 * Tv);
	return time;
}

vector<double> Get_Jerk(vector<double> Time, double Jmax, double a)
{
	vector<double> Jerk;
	for (double t = 0; t <= Time[Time.size() - 1]; t = t + SamplingTime)
	{
		double tau = 0.0;

		if (t >= Time[0] && Time[1] > t) {
			tau = Calculate_tau(t, 0, 1, Time);
			if (tau != 0)
				Jerk.push_back(Jmax / (1 + exp(-a * (1 / (1 - tau) - 1 / (tau)))));
			else
				Jerk.push_back(0);
		}
		else if (t >= Time[1] && Time[2] > t)
		{
			Jerk.push_back(Jmax);
		}
		else if (t >= Time[2] && Time[3] > t)
		{
			tau = Calculate_tau(t, 2, 3, Time);
			Jerk.push_back(Jmax / (1 + exp(a * (1 / (1 - tau) - 1 / (tau)))));
		}
		else if (t >= Time[3] && Time[4] > t)
		{
			Jerk.push_back(0);
		}

		else if (t >= Time[4] && Time[5] > t)
		{
			tau = Calculate_tau(t, 4, 5, Time);
			Jerk.push_back(-Jmax / (1 + exp(-a * (1 / (1 - tau) - 1 / (tau)))));
		}

		else if (t >= Time[5] && Time[6] > t)
		{
			Jerk.push_back(-Jmax);
		}

		else if (t >= Time[6] && Time[7] > t)
		{
			tau = Calculate_tau(t, 6, 7, Time);
			Jerk.push_back(-Jmax / (1 + exp(a * (1 / (1 - tau) - 1 / (tau)))));
		}

		else if (t >= Time[7] && Time[8] > t)
		{
			Jerk.push_back(0);
		}

		else if (t >= Time[8] && Time[9] > t)
		{
			tau = Calculate_tau(t, 8, 9, Time);
			Jerk.push_back(-Jmax / (1 + exp(-a * (1 / (1 - tau) - 1 / (tau)))));
		}

		else if (t >= Time[9] && Time[10] > t)
		{
			Jerk.push_back(-Jmax);
		}

		else if (t >= Time[10] && Time[11] > t)
		{
			tau = Calculate_tau(t, 10, 11, Time);
			Jerk.push_back(-Jmax / (1 + exp(a * (1 / (1 - tau) - 1 / (tau)))));
		}

		else if (t >= Time[11] && Time[12] > t)
		{
			Jerk.push_back(0);
		}

		else if (t >= Time[12] && Time[13] > t)
		{
			tau = Calculate_tau(t, 12, 13, Time);
			Jerk.push_back(Jmax / (1 + exp(-a * (1 / (1 - tau) - 1 / (tau)))));
		}

		else if (t >= Time[13] && Time[14] > t)
		{
			Jerk.push_back(Jmax);
		}
		else if (t >= Time[14] && Time[15] > t)
		{
			tau = Calculate_tau(t, 14, 15, Time);
			Jerk.push_back(Jmax / (1 + exp(a * (1 / (1 - tau) - 1 / (tau)))));
		}
	}
	return Jerk;
}

vector<double> Integral(vector<double> k)
{
	vector<double> result(k.size());

	for (size_t i = 1; i < k.size(); i++)
	{
		result[i] = result[i - 1] + (k[i - 1] + k[i]) / 2 * SamplingTime;
	}
	return result;
}

double Calculate_tau(double t, int a, int b, vector<double>Time)
{
	return  (t - Time[a]) / (Time[b] - Time[a]);
}

void SingleAxis(double Distance, double Smax, double &Jmax, double Amax, double Vmax, double &Ts, double &Tj, double &Ta, double &Tv)
{
	double Ts_d = pow(sqrt(3) * abs(Distance) / (8 * Smax), 1.0 / 4.0);
	double Ts_v = pow(sqrt(3) * Vmax / (2 * Smax), 1.0 / 3.0);
	double Ts_a = pow(sqrt(3) * Amax / Smax, 1.0 / 2.0);
	double Ts_j = sqrt(3) * Jmax / Smax;

	vector<double> TS = { Ts_d, Ts_v, Ts_a, Ts_j };
	double min_ = *min_element(TS.begin(), TS.end());

	if (min_ == Ts_d)
	{
		Ts = Ts_d;
		Tj = 0; Tv = 0; Ta = 0;
		Jmax = Smax * Ts / sqrt(3);
	}
	else if (min_ == Ts_v)
	{
		Ts = Ts_v;
		Tj = 0; Ta = 0;
		Calculate_Tv(Distance, Vmax, Ts, Tj, Ta, Tv);
		Jmax = Smax * Ts / sqrt(3);
	}
	else if (min_ == Ts_a)
	{
		Ts = Ts_a;
		Tj = 0;
		Jmax = Smax * Ts / sqrt(3);
		Calculate_Ta(Distance, Amax, Vmax, Ts, Tj, Ta, Tv);
	}
	else
	{
		Ts = Ts_j;
		Jmax = Jmax;
		Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts, Tj, Ta, Tv);
	}
}

void Calculate_Tv(double Distance, double Vmax, double Ts, double Tj, double Ta, double &Tv)
{
	Tv = abs(Distance) / Vmax - (4.0 * Ts + 2.0 * Tj + Ta);
}

void Calculate_Ta(double Distance, double Amax, double Vmax, double Ts, double Tj, double &Ta, double &Tv)
{
	double Ta_d = (-(6.0 * Ts + 3.0 * Tj) + sqrt(pow(2.0 * Ts + Tj, 2.0) + 4.0 * abs(Distance) / Amax)) / 2;
	double Ta_v = Vmax / Amax - 2.0 * Ts - Tj;

	vector<double> TA = { Ta_d, Ta_v };
	double min_ = *min_element(TA.begin(), TA.end());

	if (min_ == Ta_d)
	{
		Ta = Ta_d; Tv = 0.0;
	}
	else
	{
		Ta = Ta_v;
		Calculate_Tv(Distance, Vmax, Ts, Tj, Ta, Tv);
	}

}

void Calculate_Tj(double Distance, double Jmax, double Amax, double Vmax, double Ts, double &Tj, double &Ta, double &Tv)
{
	double Tj_d = pow((pow(Ts, 3.0) / 27.0 + abs(Distance) / (4.0 * Jmax) +
		sqrt(abs(Distance) * pow(Ts, 3.0) / (54.0 * Jmax) + pow(Distance, 2.0) / (16.0 * pow(Jmax, 2.0)))), (1.0 / 3.0)) +
		pow((pow(Ts, 3.0) / 27.0 + abs(Distance) / (4.0 * Jmax) -
			sqrt(abs(Distance) * pow(Ts, 3.0) / (54.0 * Jmax) + pow(Distance, 2.0) / (16.0 * pow(Jmax, 2.0)))), (1.0 / 3.0)) -
		5.0 * Ts / 3.0;

	double Tj_v = -3.0 * Ts / 2.0 + sqrt(pow(Ts, 2.0) / 4.0 + Vmax / Jmax);

	double Tj_a = Amax / Jmax - Ts;


	vector<double> TJ = { Tj_d, Tj_v, Tj_a };
	double min_ = *min_element(TJ.begin(), TJ.end());

	if (min_ == Tj_d)
	{
		Tj = Tj_d; Ta = 0.0; Tv = 0.0;
	}
	else if (min_ == Tj_v)
	{
		Tj = Tj_v; Ta = 0.0;
		Calculate_Tv(Distance, Vmax, Ts, Tj, Ta, Tv);
	}
	else
	{
		Tj = Tj_a;
		Calculate_Ta(Distance, Amax, Vmax, Ts, Tj, Ta, Tv);
	}

}
