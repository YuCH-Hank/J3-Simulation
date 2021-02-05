#include "SaveData.h"

//================================================

FILE *SaveDataFile;

//================ 建立實驗資料檔 ================
void SaveData_CreateFile(char *path)
{

	// --- 建立txt檔 ---
	/* fopen(
	string filename,		// 檔案路徑及名字
	string mode)			// "w" Open for writing only;*/
	SaveDataFile = fopen(path, "w");

}

//================ 寫入實驗資料檔 ================

void SaveData_Data(vector<double> a, vector<double> b, vector<double> c)
{
	for (int i = 0; i < a.size(); i++)
	{
		fprintf(SaveDataFile, "%f\t", a[i]);
	}
	for (int i = 0; i < b.size(); i++)
	{
		fprintf(SaveDataFile, "%f\t", b[i]);
	}
	for (int i = 0; i < c.size(); i++)
	{
		fprintf(SaveDataFile, "%f\t", c[i]);
	}
	fprintf(SaveDataFile, "\n");
}

void SaveData_Data(vector<double> a, vector<double> b, vector<double> c, vector<double> d)
{
	for (int i = 0; i < a.size(); i++)
	{
		fprintf(SaveDataFile, "%f\t", a[i]);
	}
	for (int i = 0; i < b.size(); i++)
	{
		fprintf(SaveDataFile, "%f\t", b[i]);
	}
	for (int i = 0; i < c.size(); i++)
	{
		fprintf(SaveDataFile, "%f\t", c[i]);
	}
	for (int i = 0; i < d.size(); i++)
	{
		fprintf(SaveDataFile, "%f\t", d[i]);
	}
	fprintf(SaveDataFile, "\n");
}

//================ 關閉實驗資料檔 ================

void SaveData_CloseFile()
{

	// --- 關閉txt檔 ---
	/* fclose(
	string filename)		// 檔案路徑及名字*/
	fclose(SaveDataFile);

}