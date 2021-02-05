#ifndef VECTOR_TOOL
#define VECTOR_TOOL

#include "Setting.h"
#include <algorithm>
#include <functional>

//Vector with Vector
vector<double> V_AddVector(vector<double> v, vector<double> k);
vector<double> V_AddVector(vector<double> v, vector<double> k, vector<double> t, vector<double> h);
vector<double> V_SubVector(vector<double> v, vector<double> k);
vector<double> V_MultiplyVector(vector<double> v, vector<double> k);
vector<double> V_DivideVector(vector<double> v, vector<double> k);
vector<double> V_Power(vector<double>v, int order);
vector<double> V_Sign(vector<double>v);

//Vector with Scalar
vector<double> V_ScalarDividebyVector(vector<double>v, double k);
vector<double> V_MultiplyScalar(vector<double> v, double k);
vector<double> V_AddScalar(vector<double> v, double k);
vector<double> V_SubScalar(vector<double> v, double k);

//Find Max Element
double V_min_element(vector<double>k);
double V_max_element(vector<double>k);
int V_min_index(vector<double>k);
int V_max_index(vector<double>k);
void V_min(vector<double>k, double min_, int place);
void V_max(vector<double>k, double max_, int place);
#endif