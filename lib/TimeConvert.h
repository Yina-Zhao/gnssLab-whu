////////////////////////////////////////////////
/// Shoujian Zhang, 2023, Wuhan University
//////////////////////////////////////////////
#pragma once

#include "TimeStruct.h"

// 读取跳秒
double getLeapSeconds(const CommonTime &ct);

// 时间系统转换
CommonTime convertTimeSystem(
        const CommonTime &ct,
        const TimeSystem &targetSys);

/**
 * convert from "Julian day" (= JD + 0.5)to calendar day.
 */
void convertJD2YMD(double jd,
                   int &iyear,
                   int &imonth,
                   int &iday);

/**
 *   convert calendar day to "Julian day"(= JD + 0.5)
 */
double convertYMD2JD(int iyear,
                     int imonth,
                     int iday);

/** Fundamental routine to convert seconds of day to H:M:S
 */
void convertSOD2HMS(double sod,
                    int &hh,
                    int &mm,
                    double &sec);

/** Fundamental routine to convert H:M:S to seconds of day
 * @param hh integer hour (0 <= hh < 24) (input)
 * @param mm integer minutes (0 <= mm < 60) (input)
 * @param sec double seconds (0 <= sec < 60.0) (input)
 * @return sod seconds of day (input)
 */
double convertHMS2SOD(int hh,
                      int mm,
                      double sec);

CommonTime CivilTime2CommonTime(const CivilTime &civilt);
CivilTime CommonTime2CivilTime(const CommonTime &ct);
CommonTime JulianDate2CommonTime(JulianDate &jd);
JulianDate CommonTime2JulianDate(CommonTime &ct);
void CommonTime2MJD(const CommonTime& ct, MJD& mjd );
void MJD2CommonTime(MJD& mjd, CommonTime& ct);
CommonTime YDSTime2CommonTime(YDSTime &ydst);
YDSTime CommonTime2YDSTime(const CommonTime &ct);
void CommonTime2WeekSecond(const CommonTime& ct, WeekSecond& wk);
void WeekSecond2CommonTime(WeekSecond& wk, CommonTime& ct);
