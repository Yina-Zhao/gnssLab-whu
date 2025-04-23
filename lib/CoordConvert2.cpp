/****************************************************************************
目的：    定义时间结构体及其相互转换函数,
          空间直角坐标和大地坐标相互转换

  编写时间：2011.07.12
  作者：    王甫红
  版本:     V2.0
  版权：    武汉大学测绘学院
****************************************************************************/
#include <stdio.h>
#include <math.h>
#include "CoordConvert2.h"
#include "DecodeConst.h"
#include "Matrix.h"


/****************************************************************************
XYZToBLH
目的：将空间直角坐标转换为大地坐标
编号: 05009

参数：
XYZ   空间直角坐标[m]
BLH   大地坐标[Rad, m]
R     参考椭球的长半径[m]
F     扁率(1/f)
****************************************************************************/
void XYZToBLH(const double XYZ[3], double BLH[3], const double R, const double F)
{
    short   Iterator;
    double  e2, dZ, rho2, dZ_new, SinPhi;
    double  ZdZ, Nh, N;

    N = 0.0;
    e2 = F * (2.0 - F);
    rho2 = XYZ[0] * XYZ[0] + XYZ[1] * XYZ[1];
    dZ = e2 * XYZ[2];
    dZ_new = dZ;

    Iterator = 0;
    do {
        dZ = dZ_new;
        ZdZ = XYZ[2] + dZ;
        Nh = sqrt(rho2 + ZdZ * ZdZ);

        if (Nh < 1.0)  /* 如果XYZ坐标为0.0的情况 */
        {
            BLH[0] = BLH[1] = 0.0;
            BLH[2] = -R;
            break;
        }

        SinPhi = ZdZ / Nh;
        N = R / sqrt(1.0 - e2 * SinPhi * SinPhi);
        dZ_new = N * e2 * SinPhi;

        Iterator = Iterator + 1;
    } while ((fabs(dZ - dZ_new) > 1E-8) && (Iterator < 10));

    BLH[1] = atan2(XYZ[1], XYZ[0]);
    BLH[0] = atan2(ZdZ, sqrt(rho2));
    BLH[2] = Nh - N;
}

/****************************************************************************
BLHToXYZ
目的：将大地坐标转换为空间直角坐标
编号: 05010

参数：
BLH   大地坐标[Rad, m]
XYZ   空间直角坐标[m]
R     参考椭球的长半径[m]
F     扁率(1/f)
****************************************************************************/
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F)
{
    double  e2, CosLat, SinLat;
    double  N;

    e2 = F * (2.0 - F);
    CosLat = cos(BLH[0]);
    SinLat = sin(BLH[0]);

    N = R_CGS2K / sqrt(1.0 - e2 * SinLat * SinLat);

    XYZ[0] = (N + BLH[2]) * CosLat * cos(BLH[1]);
    XYZ[1] = (N + BLH[2]) * CosLat * sin(BLH[1]);
    XYZ[2] = ((1.0 - e2) * N + BLH[2]) * SinLat;
}


void xyz2enu(const double* pos, double* E)
{
    double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

    E[0] = -sinl;      E[1] = cosl;       E[2] = 0.0;
    E[3] = -sinp * cosl; E[4] = -sinp * sinl; E[5] = cosp;
    E[6] = cosp * cosl;  E[7] = cosp * sinl;  E[8] = sinp;
}


/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
void ecef2enu(const double* pos, const double* r, double* e)
{
    double E[9], blh[3], dpos[3];

    XYZToBLH(pos, blh, R_CGS2K, F_CGS2K);    // Pos is ECEF coor of a reference station
    for (int i = 0; i < 3; i++) dpos[i] = r[i] - pos[i];
    xyz2enu(blh, E);
    MatrixMultiply(3, 3, 3, 1, E, dpos, e);
}

