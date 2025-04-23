//
// Created by shjzh on 2025/3/9.
//

#include "SPPUCCodePhase.h"
#include "StringUtils.h"
#define debug 1

#define SIG_UC_CODE 0.3
#define SIG_UC_PHASE 0.003

void SPPUCCodePhase::solve(ObsData &obsData) {
    //----------------------
    // 去掉通道号，C1W, C1C => C1;
    // 后面computeSatPos里用与通道号无关的观测值计算卫星发射时刻位置
    //----------------------
    //cout << "come in" << endl;
   // convertObsType(obsData);//todo:这里是去掉通道号，我解码时已经自动去掉了，所以这里可以共用
   // cout << "come in" << endl;
  /*  if(debug)
    {
        cout << "after convertObsType" << endl;
        cout << obsData << endl;
    }*/


    // todo
    // 探讨双频非差观测值单点定位时卫星数条件？

    //这里没有删除！这里没有删除！这里没有删除北斗！！！！
    //todo: 这里检查双频观测值是否都有，把数据缺失的卫星删除，这里也可以共用
    checkDualCodeTypes(obsData);//检查观测卫星是否具有有效的双频观测值，如果缺失或者未定义，则删除卫星
    //这里没问题了
   // cout << obsData << endl;




    // 计算发射时刻卫星位置（参考框架为时刻的）
    //todo:这里就需要改动一下了，因为获取星历的方式变了
    //todo:已经更改完
    //todo:毫无疑问，实时流的问题也出在这里
    satXvtTransTime = computeSatPos(obsData);//todo:问题在计算北斗卫星位置这里！！这一步删除了北斗卫星！！！
    //todo: 从这里的输出来看，貌似卫星位置和速度计算没有问题
    //todo: 这里的一个卫星位置速度输出是nan
   /* if(debug)
    {
       // cout << "satXvtTransTime" << CommonTime2CivilTime(obsData.epoch) << endl;
        for(auto sx: satXvtTransTime)
        {
            cout << sx.first  << endl;
            cout << sx.second << endl;
        };
    }*/

   /* if(debug)
    {
        cout << "rover>>>>:" << endl;
        cout << obsData << endl;
    }*/


    //----------------------
    // 得到卫星发射时刻位置和钟差、相对论和TGD后，改正观测值延迟，并更新C1/C2等观测值
    //----------------------
    // todo:
    // correctTGD(obsData);

    xyz = obsData.antennaPosition;
    //todo: 这里的位置也没问题
   // cout << "xyz: " << xyz << endl;
    dxyz = {100, 100, 100};

    int iter(0);
    while (true) {

        satXvtRecTime = earthRotation(xyz, satXvtTransTime);

       // satXvtRecTime = satXvtTransTime;

       /* if(debug)//todo: 这里有一个终端报错，what():  map::at，传入的键不存在
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

        //这里已经删掉了北斗卫星！这里已经删掉了北斗卫星！这里已经删掉了北斗卫星！
        int numSats = obsData.satTypeValueData.size();
       /* if(debug)
        {
            cout << "rover>>>>:" << endl;
            cout << obsData << endl;
        }
        if (debug) cout << "numSats: " << numSats << endl;*/

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
           // if(debug)
            //    cout << "computeElevAzim" << endl;

            computeElevAzim(xyz, satXvtRecTime,satElevData,satAzimData);

           /* if(debug)
            {
                cout << "satElevData:" << endl;
                cout << satElevData << endl;
            }*/

            // todo:
            // computeIonoDelay();
            // computeTropDealy();
        }

        equSys = linearize(xyz, satXvtRecTime, satElevData, obsData);

       // if(debug)
         //   cout << "afte linearize:" << endl;

        // 如果是基准站，完成线性化后就退出
        // 因为基准站位置是准确的
        if(!isRover)
            break;

        solverLsq.solve(equSys);
        dxyz = solverLsq.getdxyz();

        xyz += dxyz;

        cout
        << "iteration:"<< iter
        << "dxyz:"<< dxyz.transpose()
        << "xyz:" << xyz.transpose() << endl;


        // convergence threshold
        if (dxyz.norm() < 0.1) {
            break;
        }

        //todo: 这里有个问题，就是迭代三四次就迭代不了了
        if (iter > 20) {
            InvalidSolver e("too many iterations");
            throw(e);
        }
        iter++;

    }

    result.xyz = xyz;
}


//检查观测数据中的卫星是否具有有效的双频观测数据
void SPPUCCodePhase::checkDualCodeTypes(ObsData &obsData)  {

    SatIDSet satRejectedSet;
    // Loop through all the satellites
    for (auto &stv: obsData.satTypeValueData) {
        string sys = stv.first.system;
        // get type for current system
        std::pair<string, string> codePair;
        try {
            codePair = dualCodeTypes.at(sys);
        }
        catch (...) {
            satRejectedSet.insert(stv.first);
        }

        // 双频非组合观测值，两个频率必须同时存在，否则方程将秩亏
        if(stv.second.find(codePair.first) ==stv.second.end() ||
                stv.second.find(codePair.second) ==stv.second.end()) {
            satRejectedSet.insert(stv.first);
        }
    }
    // remove bad sat;
    for (auto sat: satRejectedSet) {
        obsData.satTypeValueData.erase(sat);
    }
};

EquSys SPPUCCodePhase::linearize(Eigen::Vector3d& xyz,
                                 std::map<SatID,Xvt>& satXvtRecTime,
                                 SatValueMap& satElevData,
                                 ObsData &obsData) {
    EquSys equSys;
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

        /*if (debug) {
            cout << "rcvPos:" << xyz << endl;
            cout << "satXYZ:" << satXYZ << endl;
            cout << "rho:" << rho << endl;
        }*/

        double clkBias = satXvtRecTime.at(sat).clkbias * C_MPS;
        double relCorr = satXvtRecTime.at(sat).relcorr * C_MPS;

        double slantTrop(0.0);
        // to do
        // extract slant trop

       /* if(debug)
        {
            cout << "clkBias:" << clkBias << endl;
            cout << "relCorr:" << relCorr << endl;
            cout << "slantTrop" << slantTrop << endl;
        }*/

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
        // 把当前观测方程未知参数插入到总体的未知参数
        varSetTemp.insert(dx);
        varSetTemp.insert(dy);
        varSetTemp.insert(dz);
        varSetTemp.insert(cdtGPS);

        // 对每个观测值，都需要存储对应的未知参数及其偏导数
        for (auto tv: stv.second)
        {
            if (sat.system == "C" && SYS == "C")
            {
                // 电离层所有频率估计的都是第一频率的伪距的电离层延迟
                Variable ionoC1G(obsData.station,
                                 sat,
                                 Parameter::iono,
                                 ObsID(sat.system, "C2"));

                // 把ionoC1G插入到观测方程
                varSetTemp.insert(ionoC1G);

                double gamma = getGamma(sat.system, "C2", "C7");

                varSetTemp.insert(ionoC1G);

                if (tv.first == "C2" )
                {
                    EquID equID = EquID(sat, tv.first);

                    //>> 先验残差
                    double prefit;
                    double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                    prefit = tv.second - computedObs;

                    /*if(debug)
                    {
                        cout << "sat:" << sat
                             << fixed << setprecision(3)
                             << "type:" << tv.first
                             << "obs:" << tv.second
                             << "rho:" << rho
                             << "clkBias:" << clkBias
                             << "relCorr:" << relCorr
                             << "prefit:" << prefit
                             << endl;
                    }*/

                    equSys.obsEquData[equID].prefit = prefit;
                    equSys.obsEquData[equID].varCoeffData[dx] = cosines[0];
                    equSys.obsEquData[equID].varCoeffData[dy] = cosines[1];
                    equSys.obsEquData[equID].varCoeffData[dz] = cosines[2];
                    equSys.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;
                    equSys.obsEquData[equID].varCoeffData[ionoC1G] = 1.0;

                    // Compute the weight according to elevation
                    double weight;
                    if(elev >= 30){
                        weight = 1.0 / (SIG_UC_CODE * SIG_UC_CODE);
                    }
                    else
                    {
                        weight = 1.0 / (SIG_UC_CODE * SIG_UC_CODE) * std::pow(std::sin(elevRad), 2);
                    }

                    equSys.obsEquData[equID].weight = weight; // IF组合方差为1.0m


                }

                else if ( tv.first == "C7" )
                {
                    EquID equID = EquID(sat, tv.first);

                    //>> 先验残差
                    double prefit;
                    double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                    prefit = tv.second - computedObs;

                    /*if(debug)
                    {
                        cout << "sat:" << sat
                             << fixed << setprecision(3)
                             << "type:" << tv.first
                             << "obs:" << tv.second
                             << "rho:" << rho
                             << "clkBias:" << clkBias
                             << "relCorr:" << relCorr
                             << "prefit:" << prefit
                             << endl;
                    }*/

                    equSys.obsEquData[equID].prefit = prefit;
                    equSys.obsEquData[equID].varCoeffData[dx] = cosines[0];
                    equSys.obsEquData[equID].varCoeffData[dy] = cosines[1];
                    equSys.obsEquData[equID].varCoeffData[dz] = cosines[2];
                    equSys.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;
                    equSys.obsEquData[equID].varCoeffData[ionoC1G] = gamma;

                    // Compute the weight according to elevation
                    double weight;
                    if(elev >= 30){
                        weight = 1.0 / (SIG_UC_CODE * SIG_UC_CODE);
                    }
                    else
                    {
                        weight = 1.0 / (SIG_UC_CODE * SIG_UC_CODE) * std::pow(std::sin(elevRad), 2);
                    }

                    equSys.obsEquData[equID].weight = weight; // IF组合方差为1.0m


                }

                else if (tv.first == "L2" )
                {
                    EquID equID = EquID(sat, tv.first);

                    //>> 先验残差
                    double prefit;
                    double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                    prefit = tv.second - computedObs;

                    /*if(debug)
                    {
                        cout << "sat:" << sat
                             << fixed << setprecision(3)
                             << "type:" << tv.first
                             << "obs:" << tv.second
                             << "rho:" << rho
                             << "clkBias:" << clkBias
                             << "relCorr:" << relCorr
                             << "prefit:" << prefit
                             << endl;
                    }*/

                    //>>>> 定义未知系数变量和系数值
                    equSys.obsEquData[equID].prefit = prefit;
                    equSys.obsEquData[equID].varCoeffData[dx] = cosines[0];
                    equSys.obsEquData[equID].varCoeffData[dy] = cosines[1];
                    equSys.obsEquData[equID].varCoeffData[dz] = cosines[2];
                    equSys.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;
                    equSys.obsEquData[equID].varCoeffData[ionoC1G] = -1.0;

                    // 定义模糊度变量
                    Variable ambL1G(obsData.station,
                                     sat,
                                     Parameter::ambiguity,
                                     ObsID(sat.system, tv.first));

                    // 将模糊度变量存储到全体变量列表中
                    varSetTemp.insert(ambL1G);

                    double wavelength = getWavelength(sat.system, safeStoi(tv.first.substr(1,1)));
                    equSys.obsEquData[equID].varCoeffData[ambL1G] = wavelength;


                    // Compute the weight according to elevation
                    double weight;
                    if(elev >= 30){
                        weight = 1.0 / (SIG_UC_PHASE * SIG_UC_PHASE);
                    }
                    else
                    {
                        weight = 1.0 / (SIG_UC_PHASE * SIG_UC_PHASE) * std::pow(std::sin(elevRad), 2);
                    }
                    equSys.obsEquData[equID].weight = weight;


                }
                else if (tv.first == "L7" )
                {
                    EquID equID = EquID(sat, tv.first);

                    //>> 先验残差
                    double prefit;
                    double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                    prefit = tv.second - computedObs;
                    equSys.obsEquData[equID].prefit = prefit;

                   /* if(debug)
                    {
                        cout << "sat:" << sat
                             << fixed << setprecision(3)
                             << "type:" << tv.first
                             << "obs:" << tv.second
                             << "rho:" << rho
                             << "clkBias:" << clkBias
                             << "relCorr:" << relCorr
                             << "prefit:" << prefit
                             << endl;
                    }*/

                    //>>>> 定义未知系数变量和系数值


                    equSys.obsEquData[equID].varCoeffData[dx] = cosines[0];
                    equSys.obsEquData[equID].varCoeffData[dy] = cosines[1];
                    equSys.obsEquData[equID].varCoeffData[dz] = cosines[2];
                    equSys.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;
                    equSys.obsEquData[equID].varCoeffData[ionoC1G] = -gamma;

                    // 定义模糊度变量
                    Variable ambL2G(obsData.station,
                                    sat,
                                    Parameter::ambiguity,
                                    ObsID(sat.system, tv.first));

                    // 将模糊度变量存储到全体变量列表中
                    varSetTemp.insert(ambL2G);

                    double wavelength = getWavelength(sat.system, safeStoi(tv.first.substr(1,1)));
                    equSys.obsEquData[equID].varCoeffData[ambL2G] = wavelength;

                    // Compute the weight according to elevation
                    double weight;
                    if(elev >= 30){
                        weight = 1.0 / (SIG_UC_PHASE * SIG_UC_PHASE);
                    }
                    else
                    {
                        weight = 1.0 / (SIG_UC_PHASE * SIG_UC_PHASE) * std::pow(std::sin(elevRad), 2);
                    }
                    equSys.obsEquData[equID].weight = weight;

                }
            }
            if(sat.system=="G" && SYS == "G")
            {
                // 电离层所有频率估计的都是第一频率的伪距的电离层延迟
                Variable ionoC1G(obsData.station,
                                 sat,
                                 Parameter::iono,
                                 ObsID(sat.system, "C1"));

                // 把ionoC1G插入到观测方程
                varSetTemp.insert(ionoC1G);

                double gamma = getGamma(sat.system, "C1", "C2");

                varSetTemp.insert(ionoC1G);

                if (tv.first == "C1" )
                {
                    EquID equID = EquID(sat, tv.first);

                    //>> 先验残差
                    double prefit;
                    double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                    prefit = tv.second - computedObs;

                    /*if(debug)
                    {
                        cout << "sat:" << sat
                             << fixed << setprecision(3)
                             << "type:" << tv.first
                             << "obs:" << tv.second
                             << "rho:" << rho
                             << "clkBias:" << clkBias
                             << "relCorr:" << relCorr
                             << "prefit:" << prefit
                             << endl;
                    }*/

                    equSys.obsEquData[equID].prefit = prefit;
                    equSys.obsEquData[equID].varCoeffData[dx] = cosines[0];
                    equSys.obsEquData[equID].varCoeffData[dy] = cosines[1];
                    equSys.obsEquData[equID].varCoeffData[dz] = cosines[2];
                    equSys.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;
                    equSys.obsEquData[equID].varCoeffData[ionoC1G] = 1.0;

                    // Compute the weight according to elevation
                    double weight;
                    if(elev >= 30){
                        weight = 1.0 / (SIG_UC_CODE * SIG_UC_CODE);
                    }
                    else
                    {
                        weight = 1.0 / (SIG_UC_CODE * SIG_UC_CODE) * std::pow(std::sin(elevRad), 2);
                    }

                    equSys.obsEquData[equID].weight = weight; // IF组合方差为1.0m


                }
                else if ( tv.first == "C2" )
                {
                    EquID equID = EquID(sat, tv.first);

                    //>> 先验残差
                    double prefit;
                    double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                    prefit = tv.second - computedObs;

                    /*if(debug)
                    {
                        cout << "sat:" << sat
                             << fixed << setprecision(3)
                             << "type:" << tv.first
                             << "obs:" << tv.second
                             << "rho:" << rho
                             << "clkBias:" << clkBias
                             << "relCorr:" << relCorr
                             << "prefit:" << prefit
                             << endl;
                    }*/

                    equSys.obsEquData[equID].prefit = prefit;
                    equSys.obsEquData[equID].varCoeffData[dx] = cosines[0];
                    equSys.obsEquData[equID].varCoeffData[dy] = cosines[1];
                    equSys.obsEquData[equID].varCoeffData[dz] = cosines[2];
                    equSys.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;
                    equSys.obsEquData[equID].varCoeffData[ionoC1G] = gamma;

                    // Compute the weight according to elevation
                    double weight;
                    if(elev >= 30){
                        weight = 1.0 / (SIG_UC_CODE * SIG_UC_CODE);
                    }
                    else
                    {
                        weight = 1.0 / (SIG_UC_CODE * SIG_UC_CODE) * std::pow(std::sin(elevRad), 2);
                    }

                    equSys.obsEquData[equID].weight = weight; // IF组合方差为1.0m


                }
                else if (tv.first == "L1" )
                {
                    EquID equID = EquID(sat, tv.first);

                    //>> 先验残差
                    double prefit;
                    double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                    prefit = tv.second - computedObs;

                    /*if(debug)
                    {
                        cout << "sat:" << sat
                             << fixed << setprecision(3)
                             << "type:" << tv.first
                             << "obs:" << tv.second
                             << "rho:" << rho
                             << "clkBias:" << clkBias
                             << "relCorr:" << relCorr
                             << "prefit:" << prefit
                             << endl;
                    }*/

                    //>>>> 定义未知系数变量和系数值
                    equSys.obsEquData[equID].prefit = prefit;
                    equSys.obsEquData[equID].varCoeffData[dx] = cosines[0];
                    equSys.obsEquData[equID].varCoeffData[dy] = cosines[1];
                    equSys.obsEquData[equID].varCoeffData[dz] = cosines[2];
                    equSys.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;
                    equSys.obsEquData[equID].varCoeffData[ionoC1G] = -1.0;

                    // 定义模糊度变量
                    Variable ambL1G(obsData.station,
                                     sat,
                                     Parameter::ambiguity,
                                     ObsID(sat.system, tv.first));

                    // 将模糊度变量存储到全体变量列表中
                    varSetTemp.insert(ambL1G);

                    double wavelength = getWavelength(sat.system, safeStoi(tv.first.substr(1,1)));
                    equSys.obsEquData[equID].varCoeffData[ambL1G] = wavelength;


                    // Compute the weight according to elevation
                    double weight;
                    if(elev >= 30){
                        weight = 1.0 / (SIG_UC_PHASE * SIG_UC_PHASE);
                    }
                    else
                    {
                        weight = 1.0 / (SIG_UC_PHASE * SIG_UC_PHASE) * std::pow(std::sin(elevRad), 2);
                    }
                    equSys.obsEquData[equID].weight = weight;


                }
                else if (tv.first == "L2" )
                {
                    EquID equID = EquID(sat, tv.first);

                    //>> 先验残差
                    double prefit;
                    double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                    prefit = tv.second - computedObs;
                    equSys.obsEquData[equID].prefit = prefit;

                   /* if(debug)
                    {
                        cout << "sat:" << sat
                             << fixed << setprecision(3)
                             << "type:" << tv.first
                             << "obs:" << tv.second
                             << "rho:" << rho
                             << "clkBias:" << clkBias
                             << "relCorr:" << relCorr
                             << "prefit:" << prefit
                             << endl;
                    }*/

                    //>>>> 定义未知系数变量和系数值


                    equSys.obsEquData[equID].varCoeffData[dx] = cosines[0];
                    equSys.obsEquData[equID].varCoeffData[dy] = cosines[1];
                    equSys.obsEquData[equID].varCoeffData[dz] = cosines[2];
                    equSys.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;
                    equSys.obsEquData[equID].varCoeffData[ionoC1G] = -gamma;

                    // 定义模糊度变量
                    Variable ambL2G(obsData.station,
                                    sat,
                                    Parameter::ambiguity,
                                    ObsID(sat.system, tv.first));

                    // 将模糊度变量存储到全体变量列表中
                    varSetTemp.insert(ambL2G);

                    double wavelength = getWavelength(sat.system, safeStoi(tv.first.substr(1,1)));
                    equSys.obsEquData[equID].varCoeffData[ambL2G] = wavelength;

                    // Compute the weight according to elevation
                    double weight;
                    if(elev >= 30){
                        weight = 1.0 / (SIG_UC_PHASE * SIG_UC_PHASE);
                    }
                    else
                    {
                        weight = 1.0 / (SIG_UC_PHASE * SIG_UC_PHASE) * std::pow(std::sin(elevRad), 2);
                    }
                    equSys.obsEquData[equID].weight = weight;

                }
            }

        }
    }
    equSys.varSet = varSetTemp;

    return equSys;
};