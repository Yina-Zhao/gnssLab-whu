//
// Created by 86191 on 2025/4/2.
//

#ifndef GNSSLAB_CSDETECTOR_H
#define GNSSLAB_CSDETECTOR_H


#include "GnssStruct.h"
#include "GnssFunc.h"

class CSDetector {
public:

    CSDetector()
            : deltaTMax(120.0), minCycles(2.0)
    {};

    VariableDataMap detect(ObsData &obsData);

    ~CSDetector(){};

    SatEpochValueMap satEpochMWData;
    SatEpochValueMap satEpochMeanMWData;
    SatEpochValueMap satEpochCSFlagData;

    // A structure used to store filter data for a SV.
    struct MWData {
        // Default constructor initializing the data in the structure,BEGINNING_OF_TIME是GPS时间系统
        MWData()
                : formerEpoch(BEGINNING_OF_TIME), windowSize(0), meanMW(0.0), varMW(0.0) {};

        CommonTime formerEpoch; ///< The previous epoch time stamp.
        int windowSize;         ///< Size of current window, in samples.
        double meanMW;          ///< Accumulated mean value of combination.
        double varMW;           ///< Accumulated std value of combination.
    };

    std::map<SatID, MWData> satMWData;


    double deltaTMax;
    double minCycles;

};

#endif //GNSSLAB_CSDETECTOR_H
