#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <float.h>
#include "Matrix.h"

/****************************************************************************
  Rotation_x

  目的：计算绕X轴的旋转矩阵
  编号：01004

  参数:
  Angle   旋转角[rad]
  Mat     3*3阶旋转矩阵
****************************************************************************/
void Rotation_x( double Angle, double Mat[] )
{
    double C, S;
    C = cos(Angle);
    S = sin(Angle);

    *(Mat+0) = 1.0;
    *(Mat+1) = 0.0;
    *(Mat+2) = 0.0;

    *(Mat+3) = 0.0;
    *(Mat+4) =   C;
    *(Mat+5) =   S;

    *(Mat+6) = 0.0;
    *(Mat+7) =  -S;
    *(Mat+8) =   C;
}

/****************************************************************************
  Rotation_y

  目的：计算绕Y轴的旋转矩阵
  编号：01005

  参数:
  Angle   旋转角[rad]
  Mat     3*3阶旋转矩阵
****************************************************************************/
void Rotation_y( double Angle, double Mat[] )
{
    double C, S;

    C = cos(Angle);
    S = sin(Angle);

    *(Mat+0) =   C;
    *(Mat+1) = 0.0;
    *(Mat+2) =  -S;

    *(Mat+3) = 0.0;
    *(Mat+4) = 1.0;
    *(Mat+5) = 0.0;

    *(Mat+6) =   S;
    *(Mat+7) = 0.0;
    *(Mat+8) =   C;
}

/****************************************************************************
  Rotation_z

  目的：计算绕Z轴的旋转矩阵
  编号：01006

  参数:
  Angle   旋转角[rad]
  Mat     3*3阶旋转矩阵
****************************************************************************/
void Rotation_z( double Angle, double Mat[] )
{
    double C, S;

    C = cos(Angle);
    S = sin(Angle);

    *(Mat+0) =   C;
    *(Mat+1) =   S;
    *(Mat+2) = 0.0;

    *(Mat+3) =  -S;
    *(Mat+4) =   C;
    *(Mat+5) = 0.0;

    *(Mat+6) = 0.0;
    *(Mat+7) = 0.0;
    *(Mat+8) = 1.0;
}

/****************************************************************************
  MatrixMultiply

  目的：矩阵相乘 M3 = M1*M2
  编号：01007

  参数:
  m1      M1的行数
  n1      M1的列数
  m2      M2的行数
  n2      M2的列数
****************************************************************************/
void MatrixMultiply( int m1, int n1, int m2, int n2,
                     const double M1[], const double M2[], double M3[] )
{
    int i, j, k;
    double Sum;

    if( (n1!=m2) || (m1<=0) || (n1<=0) || (m2<=0) || (n2<=0 ) )
    {
        printf( "Error dimension in MatrixMultiply!\n");
        exit(EXIT_FAILURE);
    }

    for ( i=0; i<m1; i++)
    {
        for ( j=0; j<n2; j++)
        {
            Sum = 0.0;

            for ( k=0; k<n1; k++)
            {
                Sum = Sum + *(M1+i*n1+k) * *(M2+k*n2+j);
            }

            *(M3+i*n2+j) = Sum;
        }
    }
}

/****************************************************************************
  MatrixMultiply_APAT

  目的：矩阵相乘 M2 = M1*P*M1T
  编号：01008

  参数:
  m1      M1的行数
  n1      M1的列数
  n       向量P的维数
  M1      输入矩阵[m1*n1]
  P       对角阵,用一维向量表示[m1],只包含对角线元素
  M2      输出矩阵[m2*n2]
****************************************************************************/
void MatrixMultiply_APAT( int m1, int n1, int n,
                          const double M1[], const double P[], double M2[] )
{
    int i, j, k;
    double Sum;

    if( (n1!=n) || (m1<=0) || (n1<=0) || (n<=0) )
    {
        printf( "Error dimension in MatrixMultiply_APAT!\n");
        exit(EXIT_FAILURE);
    }

    for ( i=0; i<m1; i++)
    {
        for ( j=0; j<m1; j++)
        {
            Sum = 0.0;

            for ( k=0; k<n1; k++)
            {
                Sum = Sum + *(M1+i*n1+k) * *(P+k) * *(M1+j*n1+k);
            }

            *(M2+i*n1+j) = Sum;
        }
    }
}


/****************************************************************************
  MatrixMultiply_APB

  目的：矩阵相乘 M3 = M1*P*M2
  编号：01009

  参数:
  m1      M1的行数
  n1      M1的列数
  m2      M2矩阵的行数[m2=n1]
  n2      M2矩阵的列数
  M1      输入矩阵[m1*n1]
  P       对角阵,用一维向量表示[m1],只包含对角线元素
  M2      输入矩阵[m2*n2]

  输出参数

  M3    输出矩阵[n*n]
****************************************************************************/
void MatrixMultiply_APB( int m1, int n1, int m2, int n2,
                         const double M1[], const double P[], const double M2[], double M3[] )
{
    int i, j, k;
    double Sum;

    if( (n1!=m2) || (m1<=0) || (n1<=0) || (m2<=0) || (n2<=0) )
    {
        printf( "Error dimension in MatrixMultiply_APB!\n");
        exit(EXIT_FAILURE);
    }

    for ( i=0; i<m1; i++)
    {
        for ( j=0; j<n2; j++)
        {
            Sum = 0.0;

            for ( k=0; k<n1; k++)
            {
                Sum = Sum + *(M1+i*n1+k) * *(P+k) * *(M2+k*n2+j);
            }

            *(M3+i*n2+j) = Sum;
        }
    }
}


/****************************************************************************
  MatrixAddition

  目的：矩阵相加 M3 = M1+M2
  编号：01010

  参数:
  m      M1的行数
  n      M1的列数
 ****************************************************************************/
void MatrixAddition( int m, int n, const double M1[], const double M2[], double M3[] )
{
    int i, j;

    if( (m<=0) || (n<=0) )
    {
        printf( "Error dimension in MatrixAddition!\n");
        exit(EXIT_FAILURE);
    }

    for( i=0; i<m; i++ )
    {
        for( j=0; j<n; j++ )
        {
            *(M3+i*n+j) = *(M1+i*n+j) + *(M2+i*n+j);
        }
    }
}

/****************************************************************************
  MatrixAddition2

  目的：矩阵相加 M2 = M1+M2
  编号：01011

  参数:
  m      M1的行数
  n      M1的列数
****************************************************************************/

void MatrixAddition2( int m, int n, const double M1[], double M2[] )
{
    int i, j;

    if( (m<=0) || (n<=0) )
    {
        printf( "Error dimension in MatrixAddition2!\n");
        exit(EXIT_FAILURE);
    }

    for( i=0; i<m; i++ )
    {
        for( j=0; j<n; j++ )
        {
            *(M2+i*n+j) = *(M1+i*n+j) + *(M2+i*n+j);
        }
    }
}

/****************************************************************************
 MatrixSubtraction

 目的：矩阵相减 M3 = M1-M2
 编号：01010

 参数:
 m      M1的行数
 n      M1的列数
 ****************************************************************************/
void MatrixSubtraction( int m, int n, const double M1[], const double M2[], double M3[] )
{
    int i, j;

    if( (m<=0) || (n<=0) )
    {
        printf( "Error dimension in MatrixSubtraction!\n");
        exit(EXIT_FAILURE);
    }

    for( i=0; i<m; i++ )
    {
        for( j=0; j<n; j++ )
        {
            *(M3+i*n+j) = *(M1+i*n+j) - *(M2+i*n+j);
        }
    }
}

/****************************************************************************
  MatrixTranspose

  目的：矩阵转置
  编号：01012

  参数:
  m      M1的行数
  n      M1的列数
  M1     输入矩阵
  MT     输出矩阵  MT = M1(T)
 ****************************************************************************/
void MatrixTranspose( int m, int n, const double M1[], double MT[] )
{
    int i, j;

    if( (m<=0) || (n<=0) )
    {
        printf( "Error dimension in MatrixTranspose!\n");
        exit(EXIT_FAILURE);
    }

    for( i=0; i<m; i++ )
    {
        for( j=0; j<n; j++ )
        {
            *(MT+j*m+i) = *(M1+i*n+j);
        }
    }
}

/****************************************************************************
SubMatrix

目的：  从FMat矩阵中获得子矩阵SubMat，子矩阵在父矩阵中是连续子块

参数:
Frow，Fcol      FMat的行数和列数, 从0开始计数
Brow，Bcol      FMat中子矩阵的起始行和起始列
row， col       子矩阵的行数和列数
FMat            父矩阵
SubMat          子矩阵

****************************************************************************/

bool SubMatrix( int Frow, int Fcol, int Brow, int Bcol, int row, int col, double FMat[], double SubMat[] )
{
    int i, j;

    if( Brow+row > Frow || Bcol+col > Fcol )
    {
        printf( "Father matrix don't contain submatrix.\n" );
        return false;
    }

    for( i=Brow; i<Brow+row; i++ )
    {
        for( j=Bcol; j<Bcol+col; j++ )
        {
            *(SubMat+(i-Brow)*col+j-Bcol) = *(FMat+i*Fcol+j);
        }
    }

    return true;
}

/****************************************************************************
SubMatrix

  目的：  将子矩阵SubMat的值赋给FMat矩阵，子矩阵在父矩阵中是连续子块

	参数:
	Frow，Fcol      FMat的行数和列数, 从0开始计数
	Brow，Bcol      FMat中子矩阵的起始行和起始列
	row， col       子矩阵的行数和列数
	FMat            父矩阵
	SubMat          子矩阵

****************************************************************************/

bool CopyMatrix( int Frow, int Fcol, int Brow, int Bcol, int row, int col, double FMat[], double SubMat[] )
{
    int i, j;

    if( Brow+row > Frow || Bcol+col > Fcol )
    {
        printf( "Father matrix don't contain submatrix.\n" );
        return false;
    }

    for( i=Brow; i<Brow+row; i++ )
    {
        for( j=Bcol; j<Bcol+col; j++ )
        {
            *(FMat+i*Fcol+j)	= *(SubMat+(i-Brow)*col+j-Bcol);
        }
    }

    return true;
}


/****************************************************************************
  AdjRow

  目的：增加或删除矩阵M的某一行

  参数:
  m      M的行数
  n      M的列数
  isAdd  1=增加行，0=删除行
  row    增加行的位置
  M      输入矩阵
 ****************************************************************************/
void AdjRow( int m, int n, int isAdd, int row, double val, double M[])
{
    int i, j, k, m1;
    double* Mat;

    m1=(isAdd==1)? (m+1) : (m-1);
    Mat = new double[m1*n];
    EmptyArray(m1*n, Mat);

    for(k=i=0;i<m;i++)
    {
        if(i==row)
        {
            if(isAdd==1)
            {
                for(j=0;j<n;j++) *(Mat+k*n+j)=val;
                k++;
            }
            else  continue;
        }

        for(j=0;j<n;j++)   *(Mat+k*n+j)=*(M+i*n+j);
        k++;
    }

    CopyArray(m1*n, M, Mat);
    delete []Mat;
}

/****************************************************************************
  AdjCol

  目的：增加或删除矩阵M的某一列

  参数:
  m      M的行数
  n      M的列数
  isAdd  1=增加列，0=删除列
  col    增加列的位置
  M      输入矩阵
 ****************************************************************************/
void AdjCol( int m, int n, int isAdd, int col, double val, double M[])
{
    int i, j, k, n1;
    double* Mat;

    n1=(isAdd==1)? (n+1) : (n-1);
    Mat = new double[m*n1];
    EmptyArray(m*n1, Mat);
    for(i=0;i<m;i++)
    {
        for(k=j=0;j<n;j++)
        {
            if(j==col)
            {
                if(isAdd==1)
                {
                    *(Mat+i*n1+k)=val; k++;
                }
                else continue;
            }
            *(Mat+i*n1+k)=*(M+i*n+j);
            k++;
        }
    }

    CopyArray(m*n1, M, Mat);
    delete []Mat;
}
/****************************************************************************
  VectDot    a = A . B

  目的：计算两个向量的点积
  编号：01013

  参数:
  m      A向量的元素个数
  n      B向量的元素个数, 要求m=n
  返回值：Val    点积
****************************************************************************/
double VectDot( int m, int n, const double A[], const double B[] )
{
    int i;
    double Val=0.0;

    if ( (m!=n) || (m<=0) || (n<=0) )
    {
        printf( "Error dimension in VectDot!\n");
        exit(EXIT_FAILURE);
    }

    for( i=0; i<m; i++ )
    {
        Val = Val + *(A+i) * *(B+i);
    }

    return (Val);
}

/****************************************************************************
 Norm

 目的：计算向量的距离
 编号：01020

 参数:
 m      A向量的元素个数
 A      向量

 返回值：Val    点间距离
 ****************************************************************************/
double Norm( const int n, const double A[] )
{
    if( n<=0 )
    {
        printf( "Error dimension in Norm!\n");
        exit(EXIT_FAILURE);
    }

    return (sqrt(VectDot(n, n, A, A)));
}

/****************************************************************************
 UnitVector

 目的：计算向量的单位向量
 编号：01020

 参数:
 m      A向量的元素个数
 A      向量
****************************************************************************/
void UnitVector( const int n,  double A[] )
{
    double dis;
    if( n<=0 )
    {
        printf( "Error dimension in UnitVector!\n");
        exit(EXIT_FAILURE);
    }

    dis = Norm(n, A);
    if(dis<DBL_EPSILON)
    {
        printf( "Divide by 0 in UnitVector!\n");
        exit(EXIT_FAILURE);
    }

    for(int i=0;i<n;i++)   A[i]/=dis;
}

/****************************************************************************
  CrossDot    C = A X B

  目的：计算两个向量的叉积
  编号：01014

  参数:
  m      A向量的元素个数
  n      B向量的元素个数, 要求m=n

  返回值：1=正常，0=致命错误
****************************************************************************/
void CrossDot( int m, int n, const double A[], const double B[], double C[] )
{
    if ( (n!=3) || (m!=3) )
    {
        printf( "Error dimension in CrossDot!\n");
        exit(EXIT_FAILURE);
    }

    C[0] = A[1]*B[2] - A[2]*B[1];
    C[1] = A[2]*B[0] - A[0]*B[2];
    C[2] = A[0]*B[1] - A[1]*B[0];
}

/****************************************************************************
 UnitCrossDot    C = A X B

 目的：计算两个向量的叉积, 在计算之前先进行单位化
 编号：01014

 参数:
 m      A向量的元素个数
 n      B向量的元素个数, 要求m=n

 返回值：1=正常，0=致命错误
 ****************************************************************************/
void UnitCrossDot( int m, int n, const double A[], const double B[], double C[] )
{
    double Len;

    if ( (n!=3) || (m!=3) )
    {
        printf( "Error dimension in UnitCrossDot!\n");
        exit(EXIT_FAILURE);
    }

    Len = Norm(m, A) * Norm(n, B);

    if( Len<DBL_EPSILON )
    {
        printf("Divided by 0 in UnitCrossDot!\n");
        exit(EXIT_FAILURE);
    }

    C[0] = (A[1]*B[2] - A[2]*B[1])/Len;
    C[1] = (A[2]*B[0] - A[0]*B[2])/Len;
    C[2] = (A[0]*B[1] - A[1]*B[0])/Len;
}

/****************************************************************************
  Dyadic    a = A . B

  目的：计算两个向量的Dyadic积，即列向量和行向量相乘，得到一个矩阵
  编号：01015

  参数:
  m      A向量的元素个数
  n      B向量的元素个数

  输出参数:
  Mat    Dyadic积矩阵[m*n]
****************************************************************************/
void Dyadic( int m, int n, const double A[], const double B[], double Mat[] )
{
    int i, j;

    if ( (m<=0) || (n<=0) )
    {
        printf( "Error dimension in Dyadic!\n");
        exit(EXIT_FAILURE);
    }

    for( i=0; i<m; i++ )
    {
        for( j=0; j<n; j++ )
        {
            *(Mat+i*n+j) = *(A+i) * *(B+j);
        }
    }
}

/****************************************************************************
  MatrixInv

  目的：矩阵求逆,采用全选主元高斯-约当法

  编号：01016

  参数:
  n      M1的行数和列数
  a      输入矩阵
  b      输出矩阵   b=inv(a)
  返回值：1=正常，0=致命错误

****************************************************************************/

int MatrixInv( int n, double a[], double b[] )
{
    int i,j,k,l,u,v,is[10],js[10];   /* matrix dimension <= 10 */
    double d, p;

    if( n<= 0 )
    {
        printf( "Error dimension in MatrixInv!\n");
        exit(EXIT_FAILURE);
    }

    /* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
    for(i=0;i<n;i++)
    {
        for(j=0;j<n;j++)
        {
            b[i*n+j]=a[i*n+j];
        }
    }

    for(k=0;k<n;k++)
    {
        d=0.0;
        for(i=k;i<n;i++)   /* 查找右下角方阵中主元素的位置 */
        {
            for(j=k;j<n;j++)
            {
                l=n*i+j;
                p = fabs(b[l]);
                if(p>d)
                {
                    d=p;
                    is[k]=i;
                    js[k]=j;
                }
            }
        }

        if(d<DBL_EPSILON)   /* 主元素接近于0，矩阵不可逆 */
        {
            printf("Divided by 0 in MatrixInv!\n");
            exit(EXIT_FAILURE);
        }

        if( is[k]!=k )  /* 对主元素所在的行与右下角方阵的首行进行调换 */
        {
            for(j=0;j<n;j++)
            {
                u=k*n+j;
                v=is[k]*n+j;
                p=b[u];
                b[u]=b[v];
                b[v]=p;
            }
        }

        if( js[k]!=k )  /* 对主元素所在的列与右下角方阵的首列进行调换 */
        {
            for( i=0; i<n; i++ )
            {
                u=i*n+k;
                v=i*n+js[k];
                p=b[u];
                b[u]=b[v];
                b[v]=p;
            }
        }

        l=k*n+k;
        b[l]=1.0/b[l];  /* 初等行变换 */
        for( j=0; j<n; j++ )
        {
            if( j!=k )
            {
                u=k*n+j;
                b[u]=b[u]*b[l];
            }
        }
        for(i=0;i<n; i++)
        {
            if(i!=k)
            {
                for(j=0; j<n; j++ )
                {
                    if( j!=k )
                    {
                        u=i*n+j;
                        b[u]=b[u]-b[i*n+k]*b[k*n+j];
                    }
                }
            }
        }
        for(i=0;i<n;i++)
        {
            if(i!=k)
            {
                u=i*n+k;
                b[u]=-b[u]*b[l];
            }
        }
    }

    for(k=n-1;k>=0;k--)  /* 将上面的行列调换重新恢复 */
    {
        if(js[k]!=k)
        {
            for(j=0;j<n;j++)
            {
                u=k*n+j;
                v=js[k]*n+j;
                p=b[u];
                b[u]=b[v];
                b[v]=p;
            }
        }
        if(is[k]!=k)
        {
            for(i=0;i<n;i++)
            {
                u=i*n+k;
                v=is[k]+i*n;
                p=b[u];
                b[u]=b[v];
                b[v]=p;
            }
        }
    }

    return (1);
}

/****************************************************************************
  mbbub

  目的：实数冒泡排序
  编号：01017

  参数:
  n      待排序序列的长度
  p      实数数组
  返回值：1=正常，0=致命错误

****************************************************************************/
void mbbub( int n, double p[] )
{
    int m, k, i, j;
    double d;

    if( n<=0 )
    {
        printf( "Error dimension in mbbub!\n");
        exit(EXIT_FAILURE);
    }

    k=0;
    m=n-1;
    while (k<m)
    {
        j=m-1;
        m=0;
        for(i=k; i<=j; i++ )
        {
            if(p[i]>p[i+1])
            {
                d=p[i];
                p[i]=p[i+1];
                p[i+1]=d;
                m=i;
            }
        }

        j=k+1;
        k=0;
        for(i=m;i>=j;i--)
        {
            if(p[i-1]>p[i])
            {
                d=p[i];
                p[i]=p[i-1];
                p[i-1]=d;
                k=i;
            }
        }
    }
}

/****************************************************************************
  CopyArray

  目的：将一个数组拷贝到另一个数组中( 多维数组均以一维表示 )
  编号：01018

  参数:
  n      拷贝的数组元素个数
  Dist   目标数组
  Sour   源数组
****************************************************************************/
void CopyArray( int n, double Dist[], const double Sour[] )
{
    int i;

    if( n<=0 )
    {
        printf( "Error dimension in CopyArray!\n");
        exit(EXIT_FAILURE);
    }

    for( i=0; i<n; i++ )
    {
        Dist[i] = Sour[i];
    }
}

/****************************************************************************
  EmptyArray

  目的：将一个数组清空，即赋0
  编号：01019

  参数:
  n      拷贝的数组元素个数
  Dist   目标数组
****************************************************************************/
void EmptyArray( int n, double Dist[] )
{
    int i;

    if( n<=0 )
    {
        printf( "Error dimension in EmptyArray!\n");
        exit(EXIT_FAILURE);
    }

    for( i=0; i<n; i++ )
    {
        Dist[i] = 0.0;
    }
}
