//
// Created by 86191 on 2025/4/7.
//

#ifndef GNSSLAB_COORDCONVERT2_H
#define GNSSLAB_COORDCONVERT2_H

#pragma once


/* 空间直角坐标,大地坐标的相互转换函数 */

void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F);
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F);

void XYZToRTN(const double State[6], const double dXYZ[3], double dRTN[3]);
void xyz2enu(const double* pos, double* E);
void ecef2enu(const double* pos, const double* r, double* e);

#endif //GNSSLAB_COORDCONVERT2_H
