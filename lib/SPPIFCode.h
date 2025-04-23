//
// Created by shjzh on 2025/3/9.
//

#ifndef GNSSLAB_SPPIFCODE_H
#define GNSSLAB_SPPIFCODE_H

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "RinexNavStore.hpp"
#include <Eigen/Eigen>

class SPPIFCode {
public:
    SPPIFCode()
    : pEphStore(NULL), isRover(true), sigIFCode(1.0), cutOffElev(10)
    {}

    void setStationAsBase()
    {
        isRover = false;
    }

    void setRinexNavStore(RinexNavStore* pStore)
    {
        pEphStore = pStore;
    };

    void setIFCodeTypes(std::map<string, std::pair<string, string>>& ifTypes)
    {
        ifCodeTypes = ifTypes;
    };

    void solve(ObsData &obsData);

    std::map<SatID,Xvt> computeSatPos(ObsData &obsData);
    Xvt computeAtTransmitTime(const CommonTime& tr,
                              const double& pr,
                              const SatID& sat);

    void computeElevAzim(Eigen::Vector3d& xyz,
                          std::map<SatID,Xvt> & satXvtTransTime,
                          SatValueMap& tempElevData,
                          SatValueMap& tempAzimData);

    void convertObsType(ObsData &obsData);
    void computeIF(ObsData &obsData);
    std::map<SatID,Xvt> earthRotation(Eigen::Vector3d& xyz,
                                      std::map<SatID,Xvt> & satXvtTransTime);
    EquSys linearize(Eigen::Vector3d& xyz,
                     std::map<SatID,Xvt>& satXvtRecTime,
                     SatValueMap& satElevData,
                     ObsData& obsData);

    EquSys getEquSys()
    {
        return equSys;
    };

    //从给定的卫星数据中找到仰角最高的卫星，并返回该卫星的标识（SatID）
    SatID getDatumSat()
    {
        double maxElev(0.0);
        SatID datumSat;
        for(auto se: satElevData)
        {
            if(se.second>maxElev)
            {
                maxElev = se.second;
                datumSat = se.first;
            }
        }
        return datumSat;
    };

    SatValueMap getSatElevData()
    {
        return satElevData;
    };

    Vector3d getXYZ()
    {
        return xyz;
    };

    Result getResult();

    ~SPPIFCode(){};

    // 继承类需要访问这个成员
protected:
    double cutOffElev;

    bool isRover;
    double sigIFCode;


    EquSys equSys;
    Result result;

    Vector3d xyz;
    Vector3d dxyz;

    std::map<SatID,Xvt> satXvtTransTime;
    std::map<SatID,Xvt> satXvtRecTime;

    SatValueMap  satElevData;
    SatValueMap  satAzimData;
    SatValueMap  satTropData;

    SolverLSQ  solverLsq;

    RinexNavStore* pEphStore;

    std::map<string, std::pair<string, string>> ifCodeTypes;

    SatID datumSat;

};


#endif //GNSSLAB_SPPIFCODE_H
