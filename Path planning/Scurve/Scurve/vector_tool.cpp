#include "vector_tool.h"

// Vector with Vector
vector<double> V_AddVector(vector<double> v, vector<double> k)
{
	transform(v.begin(), v.end(), k.begin(), v.begin(), plus<double>());
	return v;
}

vector<double> V_AddVector(vector<double> v, vector<double> k, vector<double> t, vector<double> h)
{
	vector<double> ans = V_AddVector(v, k);
	ans = V_AddVector(ans, t);
	ans = V_AddVector(ans, h);
	return ans;
}

vector<double> V_SubVector(vector<double> v, vector<double> k)
{
	transform(v.begin(), v.end(), k.begin(), v.begin(), minus<double>());
	return v;
}

vector<double> V_MultiplyVector(vector<double> v, vector<double> k)
{
	transform(v.begin(), v.end(), k.begin(), v.begin(), multiplies<double>());
	return v;
}

vector<double> V_DivideVector(vector<double> v, vector<double> k)
{
	transform(v.begin(), v.end(), k.begin(), v.begin(), divides<double>());
	return v;
}

vector<double> V_Power(vector<double> v, int order)
{
	transform(v.begin(), v.end(), v.begin(), [order](double &c) { return  pow(c, order); });
	return  v;
}

vector<double> V_Sign(vector<double>v)
{
	vector<double> sign;
	for (size_t i = 0; i < v.size(); i++)
	{
		if (v[i] >= 0)
			sign.push_back(1);
		else
			sign.push_back(-1);
	}
	return sign;
}

// Vector with Scalar
vector<double> V_ScalarDividebyVector(vector<double> v, double k)
{
	transform(v.begin(), v.end(), v.begin(), [k](double &c) { return k/c; });
	return  v;
}

vector<double> V_MultiplyScalar(vector<double> v, double k)
{
	transform(v.begin(), v.end(), v.begin(), [k](double &c) { return c*k; });
	return  v;
}

vector<double> V_AddScalar(vector<double> v, double k)
{
	transform(v.begin(), v.end(), v.begin(), [k](double &c) { return c + k; });
	return  v;
}

vector<double> V_SubScalar(vector<double> v, double k)
{
	transform(v.begin(), v.end(), v.begin(), [k](double &c) { return c - k; });
	return  v;
}

double V_min_element(vector<double> k)
{
	return *min_element(k.begin(), k.end());
}

double V_max_element(vector<double> k)
{
	return *max_element(k.begin(), k.end());
}

int V_min_index(vector<double> k)
{
	return min_element(k.begin(), k.end()) - k.begin();
}

int V_max_index(vector<double> k)
{
	return max_element(k.begin(), k.end()) - k.begin();
}

void V_min(vector<double> k, double min_, int place)
{
	min_ = V_min_element(k);
	place = min_element(k.begin(), k.end()) - k.begin();
}

void V_max(vector<double> k, double max_, int place)
{
	max_ = V_max_element(k);
	place = max_element(k.begin(), k.end()) - k.begin();
}

