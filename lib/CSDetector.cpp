//
// Created by 86191 on 2025/4/2.
//

#include "CSDetector.h"
#define debug 1

VariableDataMap CSDetector::detect(ObsData &obsData) {

    VariableDataMap csFlagData;

    // Loop through all the satellites
    CommonTime currentEpoch = obsData.epoch;
    SatIDSet badSatSet;//todo:注意这里可能又要删除坏卫星
    for (auto stv: obsData.satTypeValueData) {
        SatID sat = (stv).first;
        string L1Type, L2Type, C1Type, C2Type;
        if (sat.system == "G" && SYS == "G") {
            L1Type = "L1";
            L2Type = "L2";
            C1Type = "C1";
            C2Type = "C2";
        }
        else if(sat.system == "C" && SYS == "C") {
            L1Type = "L2";
            L2Type = "L7";
            C1Type = "C2";
            C2Type = "C7";
        }
        else // 请增加bds的处理
        {
            badSatSet.insert(sat);
        }

        // wavelengthMW of MW-combination, see LinearCombination
        double wavelengthMW = wavelengthOfMW(sat.system, L1Type, L2Type);
        double varianceMW = varOfMW(sat.system, L1Type, L2Type);

        double f1 = getFreq(sat.system, L1Type);
        double f2 = getFreq(sat.system, L2Type);

       /* if (debug) {
            cout << "f1:" << f1 << "f2:" << f2 << endl;
        }*/

        double L1Value, L2Value, C1Value, C2Value, mwValue;

        try {
            L1Value = stv.second.at(L1Type);
            L2Value = stv.second.at(L2Type);
            C1Value = stv.second.at(C1Type);
            C2Value = stv.second.at(C2Type);

            mwValue
                    = (f1 * L1Value - f2 * L2Value) / (f1 - f2)
                      - (f1 * C1Value + f2 * C2Value) / (f1 + f2);
        } catch (std::out_of_range) {
            // 无法构成mw，这个卫星观测值周跳无法探测，删除这个卫星
            badSatSet.insert(sat);
            continue; // 继续处理下一个卫星
        }

        satEpochMWData[sat][currentEpoch] = mwValue;

      /*  if (debug) {
            cout << "L1Value:" << L1Value << endl;
            cout << "L2Value:" << L2Value << endl;
            cout << "C1Value:" << C1Value << endl;
            cout << "C2Value:" << C2Value << endl;
            cout << "mwValue:" << mwValue << endl;
            cout << "wavelength:" << C_MPS / (f1 - f2) << endl;
        }*/

        //>>>>>>>>>>>>>>>>>>>>>>

        double currentDeltaT(0.0);
        double currentBias(0.0);
        double csFlag(0.0);

        currentDeltaT = (currentEpoch - satMWData[sat].formerEpoch);
        satMWData[sat].formerEpoch = currentEpoch;//这里就把当前历元赋值给先前历元，北斗时间就是北斗时间，GPST就是GPST
      /*  if (debug) {
            cout << "currentDeltaT:" << currentDeltaT << endl;
        }*/
        // Difference between current value of MW and average value
        currentBias = std::abs(mwValue - satMWData[sat].meanMW);
      /*  if (debug) {
            cout << "currentBias:" << currentBias << endl;
        }*/

        // Increment window size
        satMWData[sat].windowSize++;

        /**
         * cycle-slip condition
         * 1. if data interrupt for a given time gap, then cyce slip should be set
         * 2. if current bias is greater than 1 cycle and greater than 4 sigma of mean mw.
         */
        double sigLimit = 4 * std::sqrt(satMWData[sat].varMW);

       /* if (debug) {
            cout << "deltaTMax:" << deltaTMax << endl;
            cout << "wavelengthMW:" << wavelengthMW << endl;
            cout << "sigLimit:" << sigLimit << endl;
            cout << "minCycles:" << minCycles * wavelengthMW << endl;
        }*/

        // 波长有可能为负值
        if (currentDeltaT > deltaTMax ||
            currentBias > std::abs(minCycles * wavelengthMW) ||
            currentBias > sigLimit) {

            // reset the filter window size/meanMW/InitialVarofMW
            satMWData[sat].meanMW = mwValue;
            satMWData[sat].varMW = varianceMW;
            satMWData[sat].windowSize = 1;

          /*  if (debug) {
                cout << "* CS happened!" << endl;
            }*/
            csFlag = 1.0;
        } else {
            // MW bias from the mean value
            double mwBias(mwValue - satMWData[sat].meanMW);
            double size(static_cast<double>(satMWData[sat].windowSize));

            // Compute average
            satMWData[sat].meanMW += mwBias / size;

            // Compute variance
            // Var(i) = Var(i-1) + [ ( mw(i) - meanMW)^2/(i)- 1*Var(i-1) ]/(i);
            satMWData[sat].varMW += (mwBias * mwBias - satMWData[sat].varMW) / size;
        }

        // for print
        satEpochMeanMWData[sat][currentEpoch] = satMWData[sat].meanMW;
        satEpochCSFlagData[sat][currentEpoch] = csFlag * mwValue;  // 放大到mw数值，以方便绘图

        // 将周跳探测标志存到模糊度变量中
        Variable amb1(obsData.station,
                      sat,
                      static_cast<Parameter>(Parameter::ambiguity),
                      ObsID(sat.system, L1Type));

        Variable amb2(obsData.station,
                      sat,
                      static_cast<Parameter>(Parameter::ambiguity),
                      ObsID(sat.system, L2Type));

        csFlagData[amb1] = csFlag;
        csFlagData[amb2] = csFlag;

        //>>>>>>>>>>>>>>>>>>>>>>
    }

    // 删除坏卫星
    for (auto sat: badSatSet)
        obsData.satTypeValueData.erase(sat);

    return csFlagData;

};
