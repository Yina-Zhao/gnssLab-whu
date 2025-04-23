#pragma once
void Rotation_x(double Angle, double Mat[]);
void Rotation_y(double Angle, double Mat[]);
void Rotation_z(double Angle, double Mat[]);

void MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]);
void MatrixMultiply_APAT(int m1, int n1, int n, const double M1[], const double P[], double M2[]);
void MatrixMultiply_APB(int m1, int n1, int m2, int n2, const double M1[], const double P[], const double M2[], double M3[]);
void MatrixAddition(int m, int n, const double M1[], const double M2[], double M3[]);
void MatrixAddition2(int m, int n, const double M1[], double M2[]);
void MatrixSubtraction(int m, int n, const double M1[], const double M2[], double M3[]);
void MatrixTranspose(int m, int n, const double M1[], double MT[]);
bool SubMatrix(int Frow, int Fcol, int Brow, int Bcol, int row, int col, double FMat[], double SubMat[]);
bool CopyMatrix(int Frow, int Fcol, int Brow, int Bcol, int row, int col, double FMat[], double SubMat[]);
void AdjRow(int m, int n, int isAdd, int row, double val, double M[]);
void AdjCol(int m, int n, int isAdd, int col, double val, double M[]);

double VectDot(int m, int n, const double A[], const double B[]);
double Norm(const int n, const double A[]);
void UnitVector(const int n, double A[]);
void CrossDot(int m, int n, const double A[], const double B[], double C[]);
void UnitCrossDot(int m, int n, const double A[], const double B[], double C[]);
void Dyadic(int m, int n, const double A[], const double B[], double Mat[]);
int MatrixInv(int n, double a[], double b[]);

void mbbub(int n, double p[]);
void CopyArray(int n, double Dist[], const double Sour[]);
void EmptyArray(int n, double Dist[]);
