/****************************************************************************
目的：    定义北斗差分流动站软件需要的常量参数
编写时间：2011.07.22
作者：    王甫红
版本:     V2.0
版权：    武汉大学测绘学院
****************************************************************************/
#pragma once
#ifndef _DGPS_ROVER_CONST_H_
#define _DGPS_ROVER_CONST_H_


/* Mathematical constants  */

const double pi = 3.1415926535897932384626433832795;
#define pi2 (2.0*pi)                    /* 2pi */
#define Rad (pi/180.0)                  /* Radians per degree */
#define Deg (180.0/pi)                  /* Degrees per radian */
#define Arcs (3600.0*180.0/pi)          /* Arcseconds per radian */

/* GPS time constants  */
#define JAN61980  44244                 /* MJD of 1980.1.6 */
#define JAN11901  15385                 /* MJD of 1901.1.1 */
#define SECPERHOUR 3600.0                /* Seconds per hour */
#define SECPERDAY  86400.0                /* Seconds per day */
#define SECPERWEEK 604800.0               /* Seconds per week */

const int LeapMonths[13]   = { 0,  31,  60,  91, 121, 152, 182,
                               213, 244, 274, 305, 335, 366 };

const int NormalMonths[13] = { 0,  31,  59,  90, 120, 151, 181,
                               212, 243, 273, 304, 334, 365 };

/* General constants */
#define C_Light 299792458.0      /* Speed of light  [m/s]; IAU 1976  */

/* Physical parameters of the Earth, Sun and Moon  */
#define R_WGS84  6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563  /* Flattening; WGS-84   */
#define Omega_WGS 7.2921151467e-5   /*[rad/s], the earth rotation rate */
#define GM_Earth   398600.5e+9     /* [m^3/s^2]; WGS-84 */
#define GM_JGM3   398600.4415e+9     /* [m^3/s^2]; JGM3  */

/* Physical parameters of the Earth, Sun and Moon  */
#define R_CGS2K  6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K  1.0/298.257222101  /* Flattening; CGCS2000   */
#define Omega_BDS 7.2921150e-5      /*[rad/s], the earth rotation rate */
#define GM_BDS   398600.4418e+9     /* [m^3/s^2]; CGCS2000  */

/* some constants about GPS satellite signal */
#define  FG1_GPS  1575.42E6             /* L1信号频率 */
#define  FG2_GPS  1227.60E6             /* L2信号频率 */
#define  FG12R    (77/60.0)             /* FG1_Freq/FG2_Freq */
#define  FG12R2   (5929/3600.0)
#define  WL1_GPS  (C_Light/FG1_GPS)
#define  WL2_GPS  (C_Light/FG2_GPS)

/* some constants about Compass satellite signal */
#define  FG1_CPS  1561.098E6               /* B1信号的基准频率 */
#define  FG2_CPS  1207.140E6               /* B2信号的基准频率 */
#define  FG3_CPS  1268.520E6               /* B2信号的基准频率 */

#define  FC12R    (FG1_CPS/FG2_CPS)       /* FG1_CPS/FG2_CPS */
#define  FC12R2   (FC12R*FC12R)           /* FG1_CPS^2/FG2_CPS^2 */
#define  FC13R    (FG1_CPS/FG3_CPS)       /* FG1_CPS^2/FG3_CPS^2 */
#define  FC13R2   (FC13R*FC13R)
#define  WL1_CPS  (C_Light/FG1_CPS)
#define  WL2_CPS  (C_Light/FG2_CPS)
#define  WL3_CPS  (C_Light/FG3_CPS)

#define GPST_BDT  14         /* GPS时与北斗时的差值[s] */
#define MAXCHANNUM 36
#define MAXSATNUM  64
#define MAXGPSPRN  32
#define MAXOBSTYPENUM 9
#define MAXGEOPRN  5         /* 最大的GEO卫星号 */
#define MAXBDSPRN 63
#define MAXRAWLEN   40960



#endif  

