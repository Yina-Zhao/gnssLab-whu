//
// Created by shjzh on 2025/3/9.
//

#include "SPPIFCode.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>

#define debug 1

void SPPIFCode::solve(ObsData &obsData) {
    //----------------------
    // 去掉通道号，C1W, C1C => C1;
    // 后面computeSatPos里用与通道号无关的观测值计算卫星发射时刻位置
    //----------------------
    convertObsType(obsData);

    /*if(debug)
    {
        cout << "after convertObsType" << endl;
        cout << obsData << endl;
    }*/

    // 计算IF组合
    computeIF(obsData);

    // 计算发射时刻卫星位置（参考框架为时刻的）
    satXvtTransTime = computeSatPos(obsData);
    /*if(debug)
    {
        cout << "satXvtTransTime" << CommonTime2CivilTime(obsData.epoch) << endl;
        for(auto sx: satXvtTransTime)
        {
            cout << sx.first  << endl;
            cout << sx.second << endl;
        };
    }*/

    //----------------------
    // 得到卫星发射时刻位置和钟差、相对论和TGD后，改正观测值延迟，并更新C1/C2等观测值
    //----------------------
    // todo:
    // correctTGD(obsData);

    xyz = obsData.antennaPosition;
    dxyz = {100, 100, 100};

    int iter(0);
    while (true) {

        satXvtRecTime = earthRotation(xyz, satXvtTransTime);

        /*if(debug)
        {
            cout << "satXvtRecTime" << endl;
            for(auto sx: satXvtRecTime)
            {
                cout << sx.first << " xvt:" << endl;
                cout << sx.second << endl;
            };
        }*/

        // step 1: 确定观测值和未知参数的纬数
        // 根据数据结构中已经有的satTypePrefitData, satTypeVarCoeffData;
        // 得到numObs, numUnk的数值
        int numSats = obsData.satTypeValueData.size();

        // 这里应该抛出异常，而不是break，因为无法解算，所以后续rtk也不能算，
        // 所以在rtk的主程序里捕获这个异常，然后再continue下一个历元；
        // 如果break了，就不知道问题在哪里了
        if (numSats < 4 ) {
            SVNumException e("num of satellites is less than 4");
            throw(e);
        }


        // 地球表面才计算高度角和大气改正
        if(std::abs(xyz.norm() - RadiusEarth) < 100000.0)
        {
            satElevData.clear();
            satAzimData.clear();
           /* if(debug)
                cout << "computeElevAzim" << endl;*/

            computeElevAzim(xyz, satXvtRecTime,satElevData,satAzimData);

          /*  if(debug)
            {
                cout << "satElevData:" << endl;
                cout << satElevData << endl;
            }*/

            // todo:
            // computeIonoDelay();
            // computeTropDealy();
        }

        equSys = linearize(xyz, satXvtRecTime, satElevData, obsData);

       /* if(debug)
            cout << "afte linearize:" << endl;*/


        // 如果是基准站，完成线性化后就退出
        // 因为基准站位置是准确的
        if(!isRover)
            break;

        solverLsq.solve(equSys);
        dxyz = solverLsq.getdxyz();

        xyz += dxyz;
        cout << "iteration:" << iter
        << "dxyz:" << dxyz.transpose()
        << "xyz:" << xyz.transpose() << endl;

        // convergence threshold
        if (dxyz.norm() < 0.1) {
            break;
        }

        if (iter > 10) {
            InvalidSolver e("too many iterations");
            throw(e);
        }
        iter++;

    }

    result.xyz = xyz;
}


std::map<SatID, Xvt> SPPIFCode::computeSatPos(ObsData &obsData) {
    std::map<SatID, Xvt> satXvtData;
    SatIDSet satRejectedSet;//这里存储的是不要卫星号
    CommonTime time = obsData.epoch;
  //  cout << "Time: " << time << endl;//todo:这里没问题
    // Loop through all the satellites
    for (auto stv: obsData.satTypeValueData) {
        SatID sat(stv.first);
        Xvt xvt;
        // compute satellite ephemeris at transmitting time
        // Scalar to hold temporal value
        double obs(0.0);
        string codeType;
        if (sat.system == "G" && SYS == "G") {
            codeType = "C1";
           /* if (debug)
            {
                cout << sat << endl;
            }*/
        }
        // todo
        // 请增加bds或其他系统的观测值选择
        if (sat.system == "C" && SYS == "C")
        {
            codeType = "C2";
            /*if (debug)
            {
                cout << sat << " ";
            }*/

        }
        if (sat.system != SYS) {

           /* if (debug)
            {
                cout << sat.system << " " << SYS;
            }*/
            satRejectedSet.insert(sat);
           // if (debug) cout << 1 << endl;
            /*if (debug)
            {
                std::cout << "Rejected satellites: ";
                for (const auto& sat : satRejectedSet) {
                    std::cout << sat << " ";
                }
                std::cout << std::endl;
            }*/
            continue;
        }

        // code obs
        try {
            obs = stv.second.at(codeType);
            //todo: 这里也没问题
          /*  if(debug)
                cout << "sat:" << sat << "obs:" << codeType << "value:" << obs << endl;*/
        }
        catch (...) {//已调试，没有异常
            satRejectedSet.insert(sat);

           // if (debug) cout << 2 << endl;
            continue;
        }

        // now, compute xvt
        try {
            //todo:这里需要改变
            xvt = computeAtTransmitTime(time, obs, sat);
           // cout << "xvt: " << xvt;
        }
        catch (InvalidRequest &e) {//todo:这里有异常!!!!!!
            satRejectedSet.insert(sat);
          //  if (debug) cout << 3 << endl;
            continue;
        }
        satXvtData[sat] = xvt;
    }

    /*if(debug)
    {
        cout << "rover>>>>:" << endl;
        cout << obsData << endl;
    }*/

    // remove bad sat;
    for (auto sat: satRejectedSet) {
        obsData.satTypeValueData.erase(sat);
    }

    return satXvtData;

};

Xvt SPPIFCode::computeAtTransmitTime(const CommonTime &tr,
                                     const double &pr,
                                     const SatID &sat)
noexcept(false) {
    Xvt xvt;

    CommonTime tt;
    CommonTime transmit = tr;

  //  cout << " transmit: " << transmit << endl;
    transmit -= pr / C_MPS;
  //  cout << " transmit: " << transmit << endl;
    tt = transmit;

    // 这里也可以用while循环来替换这里的迭代次数
    for (int i = 0; i < 2; i++) {
        if (pEphStore != NULL) {
            //todo：这里需要改变，已经到最底层了！！
            //todo：实时流这里有问题！！！！
            xvt = pEphStore->getXvt(sat, tt);
           // cout << "xvt: " << xvt << endl;
        }
        tt = transmit;
        tt -= (xvt.clkbias + xvt.relcorr);
    }
    return xvt;
};

void SPPIFCode::convertObsType(ObsData &obsData) {

    SatTypeValueMap stvData;
    for (auto sd: obsData.satTypeValueData) {
        TypeValueMap tvData;
        for (auto td: sd.second) {
            tvData[td.first.substr(0, 2)] = td.second;
        }
        stvData[sd.first] = tvData;
    }

    // 替代
    obsData.satTypeValueData = stvData;
};

void SPPIFCode::computeIF(ObsData &obsData) {
    SatIDSet satRejectedSet;
    // Loop through all the satellites
    for (auto &stv: obsData.satTypeValueData) {
        string sys = stv.first.system;
        // get type for current system
        std::pair<string, string> ifPair;
        try {
            ifPair = ifCodeTypes.at(sys);
        }
        catch (...) {
            satRejectedSet.insert(stv.first);
        }

        // if组合的具体公式为：
        // if12 = (f1^2*P1 - f2^2*P2)/(f1^2-f2^2);
        cout << "computeIF:" << "sys:"
        << sys << "type1:"
        << ifPair.first
        << "type2:"
        << ifPair.second << endl;

        double f1 = getFreq(sys, ifPair.first);
        double f2 = getFreq(sys, ifPair.second);
        if(debug)
        {
            cout << "f1:" << f1 << "f2" << f2 << endl;
        }

        // 提取观测值
        // TypeID type1, type2;
        double value1, value2, ifValue;

        try {
            value1 = stv.second.at(ifPair.first);
            value2 = stv.second.at(ifPair.second);
            ifValue = (f1 * f1 * value1 - f2 * f2 * value2) / (f1 * f1 - f2 * f2);

            /*if (debug) {
                cout << "value1:" << value1 << endl;
                cout << "value2:" << value2 << endl;
                cout << "ifValue:" << ifValue << endl;
            }*/
            string ifCodeStr = "CC" + ifPair.first.substr(1, 1) + ifPair.second.substr(1, 1);

            stv.second[ifCodeStr] = ifValue;
        }
        catch (...) {
            satRejectedSet.insert(stv.first);
        }
    }

    // remove bad sat;
    for (auto sat: satRejectedSet) {
        obsData.satTypeValueData.erase(sat);
    }


};

std::map<SatID, Xvt> SPPIFCode::earthRotation(Eigen::Vector3d &xyz,
                                              std::map<SatID, Xvt> &satXvtTransTime) {

    std::map<SatID, Xvt> satXvtRecTime;
    for(auto stv: satXvtTransTime) {
        SatID sat = stv.first;
        XYZ xyzSat(stv.second.x);
        double dt = (xyzSat - xyz).norm() / C_MPS;

        double wt(0.0);
        wt = OMEGA_EARTH * dt;

        // todo:
        // Eigen中Vector3d是不是支持坐标旋转？
        // 请查询并修改

        double xSat, ySat, zSat;
        xSat = stv.second.x[0];
        ySat = stv.second.x[1];
        zSat = stv.second.x[2];

        double xSatRot(0.0), ySatRot(0.0);
        xSatRot = +std::cos(wt) * xSat + std::sin(wt) * ySat;
        ySatRot = -std::sin(wt) * xSat + std::cos(wt) * ySat;

        XYZ xyzRecTime;
        xyzRecTime[0] = xSatRot;
        xyzRecTime[1] = ySatRot;
        xyzRecTime[2] = zSat; // z轴不变

        double vxSat, vySat, vzSat;
        vxSat = stv.second.v[0];
        vySat = stv.second.v[1];
        vzSat = stv.second.v[2];

        double vxSatRot(0.0), vySatRot(0.0);
        vxSatRot = +std::cos(wt) * vxSat + std::sin(wt) * vySat;
        vySatRot = -std::sin(wt) * vxSat + std::cos(wt) * vySat;

        XYZ velRecTime;
        velRecTime[0] = vxSatRot;
        velRecTime[1] = vySatRot;
        velRecTime[2] = vzSat; // 不变

        // 替换位置和速度，得到旋转后的卫星产品
        Xvt xvtRecTime = stv.second;
        xvtRecTime.x = xyzRecTime;
        xvtRecTime.v = velRecTime;

        satXvtRecTime[sat] = xvtRecTime;
    };

    return satXvtRecTime;
};

void SPPIFCode::computeElevAzim(Eigen::Vector3d& xyz,
                                std::map<SatID,Xvt> & satXvt,
                                SatValueMap& tempElevData,
                                SatValueMap& tempAzimData
                                 )
{

    for(auto sx: satXvt)
    {
        SatID sat = sx.first;

        XYZ satXYZ = sx.second.x;

        // elevation
        double elev(0.0);
        double azim(0.0);
        elev = elevation(xyz, satXYZ);
        azim = azimuth(xyz, satXYZ);


        tempElevData[sat] = elev;
        tempAzimData[sat] = azim;
    }
};


EquSys SPPIFCode::linearize(Eigen::Vector3d& xyz,
                                  std::map<SatID,Xvt>& satXvtRecTime,
                                  SatValueMap& satElevData,
                                  ObsData &obsData) {
    EquSys equSysTemp;
    VariableSet varSetTemp;
    for (auto stv: obsData.satTypeValueData) {
        SatID sat = stv.first;

        double elev = satElevData.at(sat);
        double elevRad = elev*DEG_TO_RAD;

        // 跳过这颗卫星，不形成观测方程和未知参数数据
        if(elev < cutOffElev)
        {
            continue;
        }

        // 这里卫星的位置，应该是地球自转以后的卫星位置
        XYZ satXYZ;
        satXYZ = satXvtRecTime[sat].x;

        // rho
        double rho(0.0);
        rho = ( satXYZ - xyz).norm();
        if(debug)
        {
            cout << "Sat:" << sat << endl;
            cout << "xyz:" << xyz << endl;
            cout << "satXYZ:" << satXYZ << endl;
        }

        // convert unit form second to meter
        double clkBias = satXvtRecTime.at(sat).clkbias * C_MPS;
        double relCorr = satXvtRecTime.at(sat).relcorr * C_MPS;



        double slantTrop(0.0);
        // to do
        // extract slant trop

        // partials
        Eigen::Vector3d cosines;
        cosines[0] = (xyz.x() - satXYZ[0]) / rho;
        cosines[1] = (xyz.y() - satXYZ[1]) / rho;
        cosines[2] = (xyz.z() - satXYZ[2]) / rho;

        // todo
        // 请补充rhoDot，用于后续的单点测速

        // 首先定义所有可能的未知参数
        Variable dx(obsData.station, Parameter::dX);
        Variable dy(obsData.station, Parameter::dY);
        Variable dz(obsData.station, Parameter::dZ);
        Variable cdtGPS(obsData.station, Parameter::cdt);

        // 对每个观测值，都需要存储对应的未知参数及其偏导数
        for (auto tv: stv.second)
        {
            if (sat.system=="G" && tv.first == "CC12" )
            {
                EquID equID = EquID(sat, tv.first);

                //>> 先验残差
                double prefit;
                double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                prefit = tv.second - computedObs;

                if (debug) {
                    cout << "obs:" << tv.second << endl;
                    cout << "rho:" << rho << endl;
                    cout << "clkBias:" << clkBias << endl;
                    cout << "relCorr:" << relCorr << endl;
                    cout << "slantTrop" << slantTrop << endl;
                }

                equSysTemp.obsEquData[equID].prefit = prefit;
                equSysTemp.obsEquData[equID].varCoeffData[dx] = cosines[0];
                equSysTemp.obsEquData[equID].varCoeffData[dy] = cosines[1];
                equSysTemp.obsEquData[equID].varCoeffData[dz] = cosines[2];
                equSysTemp.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;


                // Compute the weight according to elevation
                double weight;
                if(elev >= 30){
                    weight = 1.0 / (sigIFCode * sigIFCode);
                }
                else
                {
                    weight = 1.0 / (sigIFCode * sigIFCode) * std::pow(std::sin(elevRad), 2);
                }

                equSysTemp.obsEquData[equID].weight = weight; // IF组合方差为1.0m

                // 把当前观测方程未知参数插入到总体的未知参数
                varSetTemp.insert(dx);
                varSetTemp.insert(dy);
                varSetTemp.insert(dz);
                varSetTemp.insert(cdtGPS);
            }
        }
    }

    equSysTemp.varSet = varSetTemp;
    return equSysTemp;
};