//
// Created by shjzh on 2025/1/2.
//
#include "TimeStruct.h"
#include "TimeConvert.h"

int main() {

    CivilTime civilTime(2025, 1, 4, 9, 0, 0.0);

    // convert yy/mm/dd to jd
    double jd;

    int yy = 2025;
    int month = 1;
    int day = 1;
    jd = convertYMD2JD(2025, 1, 1);

    cout << "jd for yy/mm/dd: " << yy << "/" << month << "/" << day << " is:" << fixed << jd << endl;

    JulianDate julianDate;
    CommonTime commonTime = CivilTime2CommonTime(civilTime);
    cout << "CommonTime is:" << commonTime << endl;

    julianDate = CommonTime2JulianDate(commonTime);

    cout << "julianData is:" << fixed << julianDate << endl;

    return 0;
}