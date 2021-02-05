#ifndef SAVEDATA
#define SAVEDATA

#include "Setting.h"
#include "stdio.h"

void SaveData_CreateFile(char *path);
void SaveData_CloseFile();
void SaveData_Data(vector<double> a, vector<double> b, vector<double> c);
void SaveData_Data(vector<double> a, vector<double> b, vector<double> c, vector<double> d);
#endif
