#include "SaveData.h"

//================================================

FILE *SaveDataFile;

//================ �إ߹������� ================
void SaveData_CreateFile(char *path)
{

	// --- �إ�txt�� ---
	/* fopen(
	string filename,		// �ɮ׸��|�ΦW�r
	string mode)			// "w" Open for writing only;*/
	SaveDataFile = fopen(path, "w");

}

//================ �g�J�������� ================

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

//================ ������������ ================

void SaveData_CloseFile()
{

	// --- ����txt�� ---
	/* fclose(
	string filename)		// �ɮ׸��|�ΦW�r*/
	fclose(SaveDataFile);

}