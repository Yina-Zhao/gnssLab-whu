//
// Created by shjzh on 2025/2/25.
//
#include "GnssStruct.h"

//====
// Variable
//====

// 初始化静态成员
const string Parameter::paraNameStrings[] = {
        "Unknown", "dX", "dY", "dZ", "cdt", "ifb", "iono", "ambiguity"
};

bool Variable::operator<(const Variable &right) const {
    if (station == right.station) {
        if (paraName == right.paraName) {
            if (obsID == right.obsID) {
                return (sat < right.sat);
            } else {
                return (obsID < right.obsID);
            }
        } else {
            return (paraName < right.paraName);
        }
    } else {
        return (station < right.station);
    }
}

bool Variable::operator==(const Variable &right) {
    if (station == right.station &&
        sat == right.sat &&
        obsID == right.obsID &&
        paraName == right.paraName) {
        return true;
    } else {
        return false;
    }
}

bool Variable::operator!=(const Variable &right) {
    return (!((*this) == right));
}