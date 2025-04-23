//
// Created by 86191 on 25-3-19.
//

#include <string>
#include "NavEphBDS.h"

using namespace std;

void NavEphBDS::printData() const {
    cout << "****************************************************************"
         << "************" << endl
         << "BDS Broadcast Ephemeris Data: " << endl;
    cout << "Toc: " << this->CivilToc.year << " " << this->CivilToc.month << " "
         << this->CivilToc.day << " " << this->CivilToc.hour << " "
         << this->CivilToc.minute << " " << this->CivilToc.second << endl;
    cout << scientific << setprecision(8)
         << "af0: " << setw(16) << af0 << endl
         << "af1: " << setw(16) << af1 << endl
         << "af2: " << setw(16) << af2 << endl;

    cout << "AODE: " << setw(16) << AODE << endl
         << "Crs:  " << setw(16) << Crs << endl
         << "Delta_n: " << setw(16) << Delta_n << endl
         << "M0: " << setw(16) << M0 << endl;

    cout << "Cuc: " << setw(16) << Cuc << endl
         << "ecc: " << setw(16) << ecc << endl
         << "Cus: " << setw(16) << Cus << endl
         << "sqrt_A: " << setw(16) << sqrt_A << endl;

    cout << "Toe: " << setw(16) << Toe << endl;
    cout << "Cic: " << setw(16) << Cic << endl;
    cout << "OMEGA_0: " << setw(16) << OMEGA_0 << endl;
    cout << "Cis: " << setw(16) << Cis << endl;

    cout << "i0: " << setw(16) << i0 << endl;
    cout << "Crc: " << setw(16) << Crc << endl;
    cout << "omega: " << setw(16) << omega << endl;
    cout << "OMEGA_DOT: " << setw(16) << OMEGA_DOT << endl;

    cout << "IDOT: " << setw(16) << IDOT << endl;
    cout << "Spare: " << setw(16) << "spare" << endl;
    cout << "BDSWeek: " << setw(16) << BDSWeek << endl;
    cout << "Spare: " << setw(16) << "spare" << endl;

    cout << "URA: " << setw(16) << URA << endl;
    cout << "SV_health: " << setw(16) << SV_health << endl;
    cout << "TGD1: " << setw(16) << TGD1 << endl;
    cout << "TGD2: " << setw(16) << TGD2 << endl;

    cout << "HOWtime: " << setw(16) << HOWtime << endl;
    cout << "AODC: " << setw(16) << AODC << endl;

    cout << "ctToc: " << ctToc.toString() << endl;
    cout << "ctToe: " << ctToe.toString() << endl;
}

//获取钟差
double NavEphBDS::svClockBias(const CommonTime &t) const {
     double dtc, elaptc;
     elaptc = t - ctToc;
     dtc = af0 + elaptc * (af1 + elaptc * af2);
     return dtc;
}

double NavEphBDS::svClockDrift(const CommonTime &t) const {
     double drift, elaptc;
     elaptc = t - ctToc;
     drift = af1 + elaptc * af2;
     return drift;
}

double NavEphBDS::svRelativity(const CommonTime &t) const {
     CGCS2000 ell;
     //cout <<  ell.getA()<< endl;
     ///Semi-major axis
     double A = sqrt_A * sqrt_A;//todo:这里没问题
     //if (1) cout << "sqrt_A:" << sqrt_A << endl;

     ///Computed mean motion (rad/sec)
     double n0 = std::sqrt(ell.getGM() / (A * A * A));//todo:这里的输出有不同的，因为在不同的轨道上
   //  if (1) cout << "n0: " << n0 << endl;

     ///Time from ephemeris reference epoch
     ///todo:这里的时间有问题,ctToe没有值，且加载星历的时候成功加载进去了，那么可能是t没有传参成功
     ///todo:如果t和ctToe在tk前面输出，它们都可以输出是可以输出的，说明传参没有问题
    // if (1) cout << "t:" << t << endl;
    // if (1) cout << "cToe:" << ctToe << endl;
     //todo:这里t和ctToe都是CommonTime类型，tk是double类型，但是GPS也是这样的，是可以输出的
     double tk = t - ctToe;//不用减14秒
     //if (1) cout << "tk" << tk << endl;

     if (tk > 302400) tk = tk - 604800;
     if (tk < -302400) tk = tk + 604800;



     ///Corrected mean motion
     double n = n0 + Delta_n;

     ///Mean anomaly
     double Mk = M0 + n * tk;

     //todo:这里都没有值！！！！！说明加载星历那里有点问题

    /* if (1)
     {
          cout << "A:" << A << endl;
          cout << "n0:" << n0 << endl;
          cout << "n" << n << endl;
          cout << "tk" << tk << endl;
          cout << "Mk:" << Mk << endl;
     }*/

     ///Kepler's Equation for Eccentric Anomaly
     ///solved by iteration
     double twoPI = 2.0e0 * PI;
     Mk = fmod(Mk, twoPI);
     double Ek = Mk + ecc * ::sin(Mk);
     int loop_cnt = 1;
     double F, G, delea;
     do {
          F = Mk - (Ek - ecc * ::sin(Ek));
          G = 1.0 - ecc * ::cos(Ek);
          delea = F / G;
          Ek = Ek + delea;
          loop_cnt++;
     } while ((fabs(delea) > 1.0e-11) && (loop_cnt <= 20));

     return (REL_CONST * ecc * std::sqrt(A) * ::sin(Ek));
}


//计算给定时刻北斗卫星的位置
//北斗卫星的时间基准比GPS时间快14秒。在计算北斗卫星位置时，需要在计算时间差时额外减去14秒，以统一到GPS时间基准
Xvt NavEphBDS::svXvt(const CommonTime& t) const
{
     //todo:
     //不管是什么轨道，它们都需要计算的一些东西
     Xvt sv;

     //todo:这里的椭球参数时刻以正常读取的
     CGCS2000 ell;

    /* if (1)
     {
          cout << ell.getGM() << " " << ell.getA() << " " << ell.getOmega() << endl;
     }*/


    // if (1) cout << "t:" << t << endl;//todo:这里也能输出时间，说明这个函数中传参是成功的

     //计算时钟改正
     //todo:这里就开始出问题了，没有输出值
     sv.relcorr = svRelativity(t);
     sv.clkbias = svClockBias(t);
     sv.clkdrift = svClockDrift(t);

     if (1)
     {
          cout << "关于时间的改正：" << endl;
          cout << sv.relcorr << " " << sv.clkbias << " " << sv.clkdrift << endl;
     }

     double tk = t - ctToe;//不用减14秒，因为这里转换成了北斗时间系统
     if (tk > 302400) tk = tk - 604800;
     if (tk < -302400) tk = tk + 604800;



     //MEO和IGSO轨道
     if ((i0 * RAD_TO_DEG <= 60) && (i0 * RAD_TO_DEG >= 50))
     {
          //todo:
          //北斗卫星位置与速度，计算方法与gps卫星相似
          // 半长轴
          double A = sqrt_A * sqrt_A;

          // 平均运动
          double n0 = std::sqrt(ell.getGM() / (A * A * A));

          // 修正后的平均运动
          double n = n0 + Delta_n;

          // 平近点角
          double Mk = M0 + n * tk;

          // 解开普勒方程求解偏近点角
          double twoPI = 2.0e0 * PI;
          Mk = fmod(Mk, twoPI);
          double Ek = Mk + ecc * ::sin(Mk);
          int loop_cnt = 1;
          double F, G, delea;
          do {
               F = Mk - (Ek - ecc * ::sin(Ek));
               G = 1.0 - ecc * ::cos(Ek);
               delea = F / G;
               Ek = Ek + delea;
               loop_cnt++;
          } while ((fabs(delea) > 1.0e-11) && (loop_cnt <= 20));

          ///True Anomaly
          double q = std::sqrt(1.0 - ecc * ecc);
          double sinEk = ::sin(Ek);
          double cosEk = ::cos(Ek);

          double GSTA = q * sinEk;
          double GCTA = cosEk - ecc;
          double vk = atan2(GSTA, GCTA);

          ///Argument of Latitude
          double phi_k = vk + omega;
          double cos2phi_k = ::cos(2.0 * phi_k);
          double sin2phi_k = ::sin(2.0 * phi_k);

          double duk = cos2phi_k * Cuc + sin2phi_k * Cus;
          double drk = cos2phi_k * Crc + sin2phi_k * Crs;
          double dik = cos2phi_k * Cic + sin2phi_k * Cis;

          double uk = phi_k + duk;
          double rk = A * (1.0 - ecc * cosEk) + drk;
          double ik = i0 + dik + IDOT * tk;

          ///Positions in orbital plane.
          double xip = rk * ::cos(uk);
          double yip = rk * ::sin(uk);

          ///Corrected longitude of ascending node.
          double OMEGA_k = OMEGA_0 + (OMEGA_DOT - ell.getOmega()) * tk
                           - ell.getOmega() * Toe;

          ///Earth-fixed coordinates.
          double sinOMG_k = ::sin(OMEGA_k);
          double cosOMG_k = ::cos(OMEGA_k);
          double cosik = ::cos(ik);
          double sinik = ::sin(ik);

          double xef = xip * cosOMG_k - yip * cosik * sinOMG_k;
          double yef = xip * sinOMG_k + yip * cosik * cosOMG_k;
          double zef = yip * sinik;
          sv.x[0] = xef;
          sv.x[1] = yef;
          sv.x[2] = zef;

          /// Compute velocity of rotation coordinates
          double dek, dlk, div, domk, duv, drv, dxp, dyp;
          dek = n * A / rk;
          dlk = sqrt_A * q * std::sqrt(ell.getGM()) / (rk * rk);
          div = IDOT - 2.0e0 * dlk * (Cic * sin2phi_k - Cis * cos2phi_k);
          domk = OMEGA_DOT - ell.getOmega();
          duv = dlk * (1.e0 + 2.e0 * (Cus * cos2phi_k - Cuc * sin2phi_k));
          drv = A * ecc * dek * sinEk - 2.e0 * dlk * (Crc * sin2phi_k - Crs * cos2phi_k);
          dxp = drv * ::cos(uk) - rk * ::sin(uk) * duv;
          dyp = drv * ::sin(uk) + rk * ::cos(uk) * duv;

          /// Calculate velocities
          double vxef = dxp * cosOMG_k - xip * sinOMG_k * domk - dyp * cosik * sinOMG_k
                        + yip * (sinik * sinOMG_k * div - cosik * cosOMG_k * domk);
          double vyef = dxp * sinOMG_k + xip * cosOMG_k * domk + dyp * cosik * cosOMG_k
                        - yip * (sinik * cosOMG_k * div + cosik * sinOMG_k * domk);
          double vzef = dyp * sinik + yip * cosik * div;

          sv.v[0] = vxef;
          sv.v[1] = vyef;
          sv.v[2] = vzef;

         /* if (1)
          {
               cout << " MEO/IGSO轨道上北斗卫星位置与速度：" << endl;
               cout << sv << endl;
          }*/


     }

     //GEO轨道,轨道倾角接近0°
     else if ((i0 * RAD_TO_DEG < 30) && (i0 * RAD_TO_DEG >= 0))
     {
          //北斗卫星位置与速度
          ///Corrected longitude of ascending node.
          double OMEGA_k = OMEGA_0 + (OMEGA_DOT - ell.getOmega()) * tk
                           - ell.getOmega() * Toe;

          //计算卫星在轨道平面内的位置
          double r = 35786000;//地球同步轨道半径(米)
          double X_k = r * ::cos(OMEGA_k);
          double Y_k = r * ::sin(OMEGA_k);
          double Z_k = 0;

          //考虑地球自转的影响，计算卫星实际位置
          double xef = X_k * ::cos(ell.getOmega() * tk) + Y_k * ::cos(-5 * DEG_TO_RAD) * ::sin(ell.getOmega() * tk);
          double yef = -X_k * ::sin(ell.getOmega() * tk) + Y_k * ::cos(-5 * DEG_TO_RAD) * ::cos(ell.getOmega() * tk);
          double zef = Y_k * ::cos(-5 * DEG_TO_RAD);

          sv.x[0] = xef;
          sv.x[1] = yef;
          sv.x[2] = zef;

          //计算卫星速度
          double vxef = -ell.getOmega() * yef;
          double vyef = ell.getOmega() * xef;
          double vzef = 0;

          sv.v[0] = vxef;
          sv.v[1] = vyef;
          sv.v[2] = vzef;

         /* if (1)
          {
               cout << " GEO轨道上北斗卫星位置与速度：" << endl;
               cout << sv << endl;
          }*/

     }

     return sv;
}







double NavEphBDS::svURA(const CommonTime &t) const {
     double ephURA = URA;
     return ephURA;
}

bool NavEphBDS::isValid(const CommonTime &ct) const {
     if (ct < beginValid || ct > endValid) return false;
     return true;
}
