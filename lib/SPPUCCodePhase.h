//
// Created by shjzh on 2025/3/9.
//

#ifndef GNSSLAB_SPPUCCODEPHASE_H
#define GNSSLAB_SPPUCCODEPHASE_H
#include "SPPIFCode.h"

class SPPUCCodePhase: public SPPIFCode {
public:
    SPPUCCodePhase()
    {};

    EquSys linearize(Eigen::Vector3d& xyz,
                     std::map<SatID,Xvt>& satXvtRecTime,
                     SatValueMap& satElevData,
                     ObsData& obsData);

    void solve(ObsData &obsData);

    void checkDualCodeTypes(ObsData &obsData);

    void setDualCodeTypes(std::map<string, std::pair<string, string>>& types)
    {
        dualCodeTypes = types;
    };

    std::map<string, std::pair<string, string>> dualCodeTypes;

};


#endif //GNSSLAB_SPPUCCODEPHASE_H
