//
// Created by 86191 on 25-3-19.
//
#pragma once
#ifndef NAVEPHBDS_H
#define NAVEPHBDS_H

#include "GnssStruct.h"
#include <cmath>

class NavEphBDS {
public:
    /// Default constructor
    NavEphBDS(void)
        : beginValid(END_OF_TIME),
          endValid(BEGINNING_OF_TIME) {
        beginValid.m_timeSystem = TimeSystem::BDT;  // 使用北斗时间系统
        endValid.m_timeSystem = TimeSystem::BDT;
    }

    /// Destructor
    virtual ~NavEphBDS(void) {}

    /// Dump the overhead information to the given output stream.
    /// throw Invalid Request if the required data has not been stored.
    void printData() const;

    /// Compute the satellite clock bias (seconds) at the given time
    /// throw Invalid Request if the required data has not been stored.
    double svClockBias(const CommonTime &t) const;

    /// Compute the satellite clock drift (sec/sec) at the given time
    /// throw Invalid Request if the required data has not been stored.
    double svClockDrift(const CommonTime &t) const;

    /// Compute satellite relativity correction (sec) at the given time
    /// throw Invalid Request if the required data has not been stored.
    double svRelativity(const CommonTime &t) const;

    /// Return URA of broadcast
    double svURA(const CommonTime &t) const;

    /// Compute satellite position at the given time.
    Xvt svXvt(const CommonTime &t) const;

    /// Check if the ephemeris is valid at the given time.
    bool isValid(const CommonTime &ct) const;

    /// Ephemeris data
    ///   SV/EPOCH/SV CLK
    int prn = 0;
    CivilTime CivilToc;        ///< Time of clock (year/month/day/hour/min/sec BDT)
    double Toc;                ///< Time of clock (seconds of BDT week)
    double af0;                ///< SV clock bias (seconds)
    double af1;                ///< SV clock drift (sec/sec)
    double af2;                ///< SV clock drift rate (sec/sec2)

    ///   BROADCAST ORBIT-1
    double AODE;               ///< Age of Data, Ephemeris (北斗使用 AODE 而不是 IODE),用来检查数据是否有效
    double Crs;                ///< Amplitude of the sine harmonic correction term to the orbit radius (meters)
    double Delta_n;            ///< Mean Motion Difference From Computed Value (semi-circles/sec)
    double M0;                 ///< Mean Anomaly at Reference Time (semi-circles)

    ///   BROADCAST ORBIT-2
    double Cuc;                ///< Amplitude of the cosine harmonic correction term to the argument of latitude (radians)
    double ecc;                ///< Eccentricity
    double Cus;                ///< Amplitude of the sine harmonic correction term to the argument of latitude (radians)
    double sqrt_A;             ///< Square Root of the Semi-Major Axis (sqrt(m))

    ///   BROADCAST ORBIT-3
    double Toe;                ///< Time of Ephemeris (seconds of BDT week)
    double Cic;                ///< Amplitude of the cosine harmonic correction term to the angle of inclination (radians)
    double OMEGA_0;            ///< Longitude of Ascending Node of Orbit Plane (semi-circles)
    double Cis;                ///< Amplitude of the sine harmonic correction term to the angle of inclination (radians)

    ///   BROADCAST ORBIT-4
    double i0;                 ///< Inclination Angle at Reference Time (semi-circles)
    double Crc;                ///< Amplitude of the cosine harmonic correction term to the orbit radius (meters)
    double omega;              ///< Argument of Perigee (semicircles)
    double OMEGA_DOT;          ///< Rate of Right Ascension (semicircles/sec)

    ///   BROADCAST ORBIT-5
    double IDOT;               ///< Rate of Inclination Angle (semi-circles/sec)

    double spare1;   //spare

    double BDSWeek;            ///< BDT week number (continuous number, not mod 1024)
    double spare2 ;            ///< L2 P code flag (not used in BDS),spare

    ///   BROADCAST ORBIT-6
    double URA;                ///< SV accuracy (meters) See BDS ICD
    double SV_health;          ///< SV health status (bits 17-22 with 3 sf 1)
    double TGD1;               ///< Group delay differential for B1 signal (seconds)
    double TGD2;               ///< Group delay differential for B2 signal (seconds)


    ///   BROADCAST ORBIT-7
    long HOWtime;              ///< Transmission time of message (seconds of BDT week)
    double AODC;               ///< Age of Data, Clock (北斗使用 AODC 而不是 IODC)
     //spare
     //spare

    /// Member data
    CommonTime ctToc;          ///< Toc in CommonTime form
    CommonTime ctToe;          ///< Toe in CommonTime form
    CommonTime transmitTime;   ///< Transmission time in CommonTime form
    CommonTime beginValid;     ///< Time at beginning of validity
    CommonTime endValid;       ///< Time at end of fit validity

    double		ClkBias;
    double		ClkDrift;
    double		ClkDriftRate;

private:
    /// Get the fit interval in hours from the fit interval flag and the AODC
    static short getFitInterval(const short AODC, const short fitIntFlag);

}; // end class NavEphBDS

#endif //NAVEPHBDS_H
