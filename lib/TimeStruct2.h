#pragma once

struct COMMONTIME   /* 通用时间定义 */
{
    unsigned short Year;
    unsigned short Month;
    unsigned short Day;
    unsigned short Hour;
    unsigned short Minute;
    double         Second;

    COMMONTIME()
    {
        Year = 0;
        Month = 0;
        Day = 0;
        Hour = 0;
        Minute = 0;
        Second = 0.0;
    }
};

struct GPSTIME              /* GPS时间定义 */
{
    unsigned short Week;
    double         SecOfWeek;

    GPSTIME()
    {
        Week = 0;
        SecOfWeek = 0.0;
    }
};

struct MJDTIME             /* 简化儒略日 */
{
    int Days;
    double FracDay;

    MJDTIME()
    {
        Days = 0;
        FracDay = 0.0;
    }
};

/* 年积日结构体定义*/
struct DOYTIME
{
    short Year;
    short Doy;
    double Fracday;

    DOYTIME()
    {
        Year = 0;
        Doy = 0;
        Fracday = 0.0;
    }
};