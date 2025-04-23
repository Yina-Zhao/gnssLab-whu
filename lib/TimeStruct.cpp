//
// Created by shjzhang on 23-12-29.
//

#include "TimeStruct.h"
#include "Exception.h"
#include <limits>
#include <math.h>

const string TimeSystem::sys_strings[count] = {
        "Unknown",
        "GPS", // GPS system time
        "GLO", // glonass system time
        "GAL", // Galileo system time
        "BDT", // BDS system time
        "QZS", // qzss time
        "UTC", // utc
        "TAI",
        "TT"
};

CivilTime &CivilTime::operator=(const CivilTime &right) {
    year = right.year;
    month = right.month;
    day = right.day;
    hour = right.hour;
    minute = right.minute;
    second = right.second;
    timeSys = right.timeSys;
    return *this;
}

bool CivilTime::operator==(const CivilTime &right) const {
    if (timeSys != right.timeSys)
        return false;

    if (year == right.year &&
        month == right.month &&
        day == right.day &&
        hour == right.hour &&
        minute == right.minute &&
        fabs(second - right.second) < std::numeric_limits<double>::epsilon()) {
        return true;
    }
    return false;
}

//=============================
JulianDate &JulianDate::operator=(const JulianDate &right) {
    jd = right.jd;
    timeSystem = right.timeSystem;
    return *this;
}

void JulianDate::reset() {
    jd = 0.0;
    timeSystem = TimeSystem::GPS;
}

bool JulianDate::operator==(const JulianDate &right) const {

    if (timeSystem != right.timeSystem)
        return false;

    if (fabs(jd - right.jd) < std::numeric_limits<double>::epsilon()) {
        return true;
    }
    return false;
}

bool JulianDate::operator!=(const JulianDate &right) const {
    return (!operator==(right));
}

bool JulianDate::operator<(const JulianDate &right) const {
    if (timeSystem != right.timeSystem) {
        InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
        throw (ir);
    }

    if (jd < right.jd) {
        return true;
    }
    return false;
}

//==================================

YDSTime &YDSTime::operator=(const YDSTime &right) {
    year = right.year;
    doy = right.doy;
    sod = right.sod;
    timeSystem = right.timeSystem;
    return *this;
}

void YDSTime::reset() {
    year = doy = 0;
    sod = 0.0;
    timeSystem = TimeSystem::GPS;
}

bool YDSTime::operator==(const YDSTime &right) const {
    /// Any (wildcard) type exception allowed, otherwise must be same time systems
    if (timeSystem != right.timeSystem)
        return false;

    if (year == right.year &&
        doy == right.doy &&
        fabs(sod - right.sod) < std::numeric_limits<double>::epsilon()) {
        return true;
    }
    return false;
}

bool YDSTime::operator!=(const YDSTime &right) const {
    return (!operator==(right));
}

bool YDSTime::operator<(const YDSTime &right) const {
    if (timeSystem != right.timeSystem) {
        InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
        throw (ir);
    }

    if (year < right.year) {
        return true;
    }
    if (year > right.year) {
        return false;
    }
    if (doy < right.doy) {
        return true;
    }
    if (doy > right.doy) {
        return false;
    }
    if (sod < right.sod) {
        return true;
    }
    return false;
}

MJD &MJD::operator=(const MJD &right) {
    mjd = right.mjd;
    timeSystem = right.timeSystem;
    return *this;
}

void MJD::reset() {
    mjd = 0.0;
    timeSystem = TimeSystem::GPS;
}

bool MJD::operator==(const MJD &right) const {
    /// Any (wildcard) type exception allowed, otherwise must be same time systems
    if (timeSystem != right.timeSystem)
        return false;

    if (fabs(mjd - right.mjd) < 4.*std::numeric_limits<double>::epsilon()) {
        return true;
    }
    return false;
}

bool MJD::operator!=(const MJD &right) const {
    return (!operator==(right));
}

bool MJD::operator<(const MJD &right) const {
    /// Any (wildcard) type exception allowed, otherwise must be same time systems
    if (timeSystem != right.timeSystem) {
        InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
        throw(ir);
    }

    if (mjd < right.mjd) {
        return true;
    }
    return false;
}

bool MJD::operator>(const MJD &right) const {
    return (!operator<=(right));
}

bool MJD::operator<=(const MJD &right) const {
    return (operator<(right) ||
            operator==(right));
}

bool MJD::operator>=(const MJD &right) const {
    return (!operator<(right));
}


WeekSecond &WeekSecond::operator=(const WeekSecond &right) {
    week = right.week;
    timeSystem = right.timeSystem;
    sow = right.sow;
    return *this;
}


void WeekSecond::reset() {
    week = 0.0;
    timeSystem = TimeSystem::GPS;
    sow = 0.0;
}

bool WeekSecond::operator==(const WeekSecond &right) const {
    return (week == right.week &&
            timeSystem == right.timeSystem &&
            sow == right.sow);
}

bool WeekSecond::operator!=(const WeekSecond &right) const {
    return (!operator==(right));
}

bool WeekSecond::operator<(const WeekSecond &right) const {
    if (week < right.week) {
        return true;
    }
    if (week > right.week) {
        return true;
    }
    if (sow < right.sow) {
        return true;
    }
    return false;
}

bool WeekSecond::operator>(const WeekSecond &right) const {
    return (!operator<=(right));
}

bool WeekSecond::operator<=(const WeekSecond &right) const {
    return (operator<(right) || operator==(right));
}

bool WeekSecond::operator>=(const WeekSecond &right) const {
    return (!operator<(right));
}








































