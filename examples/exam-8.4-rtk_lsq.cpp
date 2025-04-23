//
// Created by shjzh on 2025/2/25.
//

#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include <set>
#include "GnssStruct.h"
#include "TimeConvert.h"
#include "GnssFunc.h"
#include "RinexNavStore.hpp"
#include "RinexObsReader.h"
#include "SPPUCCodePhase.h"
#include "ARLambda.hpp"

using namespace std;

#define debug 1



int main() {

    //--------------------
    // 打开文件流
    //--------------------

    // Replace with your actual RINEX file path
    string dirPath = "D:\\rtk\\gnssLab-2.2\\data\\Zero-baseline\\";

    // rover obs file name
    std::string roverFile = dirPath + "oem719-202203031500-1.obs";//流动站观测值文件
    std::string baseFile = dirPath + "oem719-202203031500-2.obs";//基准站观测值文件
    //cout << roverFile << endl;

    // nav file name, download from IGS ftp site:ftp://gssc.esa.int/gnss/data/daily/YYYY/brdc
    std::string navFile = dirPath + "BRDC00IGS_R_20220620000_01D_MN.rnx";

    std::fstream roverObsStream(roverFile);//打开流动站观测值文件，并检查是否打开成功
    if (!roverObsStream) {
        cerr << "rover file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    std::fstream baseObsStream(baseFile);//打开基准站观测值文件，并检查是否打开成功
    if (!baseObsStream) {
        cerr << "base file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    // read nav file data before rtk
    RinexNavStore navStore;//已经加入北斗系统
    //todo:这里加载北斗星历数据时没有问题的，是卫星位置计算有问题
    navStore.loadFile(navFile);//直接加载整个导航电文文件，已经加入北斗系统

    std::map<string, std::set<string>> selectedTypes;//已经加入北斗系统
    selectedTypes["G"].insert("C1C");
    selectedTypes["G"].insert("C2W");
    selectedTypes["G"].insert("L1C");
    selectedTypes["G"].insert("L2W");
    selectedTypes["C"].insert("C2I");
    selectedTypes["C"].insert("C7I");
    selectedTypes["C"].insert("L2I");
    selectedTypes["C"].insert("L7I");





    std::map<string, std::pair<string, string>> ifCodeTypes;//已经加入北斗系统
    ifCodeTypes["G"].first = "C1";
    ifCodeTypes["G"].second = "C2";
    ifCodeTypes["C"].first = "C2";
    ifCodeTypes["C"].second = "C7";

    //-------------------
    // 定义数据处理的对象
    //-------------------
    //>>> classes for rover
    RinexObsReader readObsRover;//这里观测值文件读取包含了北斗系统和gps系统
    readObsRover.setFileStream(&roverObsStream);
    readObsRover.setSelectedTypes(selectedTypes);

    std::map<string, std::pair<string, string>> dualCodeTypes;//双频观测数据，已经加入北斗系统
    dualCodeTypes["G"].first = "C1";
    dualCodeTypes["G"].second = "C2";
    dualCodeTypes["C"].first = "C2";
    dualCodeTypes["C"].second = "C7";

    SPPUCCodePhase sppUCCodePhaseRover;
    sppUCCodePhaseRover.setRinexNavStore(&navStore);
    sppUCCodePhaseRover.setDualCodeTypes(dualCodeTypes);

    //>>> classes for base
    RinexObsReader readObsBase;//这里观测值文件读取包含了北斗系统和gps系统
    readObsBase.setFileStream(&baseObsStream);
    readObsBase.setSelectedTypes(selectedTypes);
    
    SPPUCCodePhase sppUCCodePhaseBase;
    sppUCCodePhaseBase.setRinexNavStore(&navStore);//同流动站
    sppUCCodePhaseBase.setStationAsBase();//isRover = false;
    sppUCCodePhaseBase.setDualCodeTypes(dualCodeTypes);//同流动站

    //>>> classes for rtk;    
    SolverLSQ solverRTK;//估计接收机的位置参数。通过最小化观测值与理论值之间的残差平方和，最小二乘法能够提供最优的参数估计
    //这里默认是gps,需要扩展到bds

    //定义一个截止历元，方便调试
    CivilTime stopCivilTime = CivilTime(2022, 03, 03, 06, 48, 37);
    CommonTime stopEpoch = CivilTime2CommonTime(stopCivilTime);

    std::string solFile = roverFile + ".rtk.gps.out";//这个文件里面只有gps的单点定位和rtk浮点解
    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        cerr << "solution file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    std::string solFileFixed = roverFile + ".rtk.gps.fixed.out";//这个文件里面只有gps的单点定位和rtk固定解
    std::fstream solStreamFixed(solFileFixed, ios::out);
    if (!solStreamFixed) {
        cerr << "solution file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    std::string solFileBDS = roverFile + ".rtk.bds.out";//这个文件里面只有bds的单点定位和rtk浮点解
    std::fstream solStreamBDS(solFileBDS, ios::out);
    if (!solStreamBDS)
    {
        cerr << "solution file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    std::string solFileBDSFixed = roverFile + ".rtk.bds.fixed.out";
    std::fstream solStreamBDSFixed(solFileBDSFixed, ios::out);
    if (!solStreamBDSFixed)
    {
        cerr << "solution file open error!" << strerror(errno) << endl;
        exit(-1);
    }


    while (true) {

        // solve spp for rover，流动站的单点定位
        ObsData roverData;

        try {
            roverData = readObsRover.parseRinexObs();//读取一个历元
        }
        catch (EndOfFile &e) { break; }

       /* if(debug)
        {
            cout << "rover>>>>:" << endl;
            cout << roverData << endl;
        }*/

        CommonTime epoch = roverData.epoch;

        // 调试代码时，设置一个stopEpoch，有助于快速得到结果
        if (roverData.epoch > stopEpoch)
            break;

        try
        {
            sppUCCodePhaseRover.solve(roverData);//之前有了广播星历的数据了,这个函数没有返回值，但是给xyz赋值
        }catch (SVNumException &e)
        {
            cout << "rover less 4";
            continue;
        }

        EquSys equSysRover = sppUCCodePhaseRover.getEquSys();//获取观测方程
        SatID datumSat = sppUCCodePhaseRover.getDatumSat();//SatID需要实现bd2和bd3的区分，这里是找到仰角最高的卫星
        Vector3d xyzRover = sppUCCodePhaseRover.getXYZ();//这个函数就是用来获取solve()函数得到的xyz值，是单点定位的坐标值，精度不够高。

        // solve spp for base
        ObsData baseObsData;
        try {
            baseObsData = readObsBase.parseRinexObs(epoch);
        }
        // 同步出现错误，跳过基准站处理，读取下一个流动站历元
        catch (SyncException &e) {
            continue;
        };

        /*if(debug)
        {
            cout << "Base data at epoch:"  << CommonTime2CivilTime(baseObsData.epoch) << endl;
            cout << baseObsData << endl;
        }*/

        try
        {
            sppUCCodePhaseBase.solve(baseObsData);
        }catch (SVNumException &e) {continue;}


        EquSys equSysBase = sppUCCodePhaseBase.getEquSys();

        // compute between-station single-difference equation
        EquSys equSysSD;
        differenceStation(equSysRover, equSysBase, equSysSD);//站间单差

        // compute between-sat single-difference equation
        EquSys equSysDD;
        differenceSat(datumSat, equSysSD, equSysDD);

        // solve solution
        solverRTK.solve(equSysDD);
        Vector3d dxyzRTK = solverRTK.getdxyz();

        //if(debug)
          //  cout<< dxyzRTK << endl;

        // 获得rtk定位后的接收机位置
        // 其数值应该为接收机单点定位位置+rtk定位后的dxyz；
        Vector3d xyzRTKFloat;
        xyzRTKFloat = xyzRover + dxyzRTK;

        // print float solution to files

        if (SYS == "G")printSolution(solStream, epoch, xyzRover, xyzRTKFloat);
        if (SYS == "C")printSolution(solStreamBDS, epoch, xyzRover, xyzRTKFloat);

        //todo:以上，北斗和GPS都运行成功
        
        // fix float solution to fixed ones
        int rows = solverRTK.getCovMatrix().rows();
        int cols = solverRTK.getCovMatrix().cols();
        int size = solverRTK.getState().size();
        VectorXd stateVec = VectorXd::Zero(size);  // 全0向量
        MatrixXd covMatrix = MatrixXd::Zero(rows, cols);  // 全零矩阵

        stateVec = solverRTK.getState();//定义别名，行数动态大小的、只有1列的列向量
        covMatrix = solverRTK.getCovMatrix();//定义别名，行数和列数都是动态的


       /* if (1)
        {
            cout << "stateVec.size(): " << stateVec.size() << endl;
            cout << "covMatrix.rows: " << covMatrix.rows() << endl;
            cout << "covMatrix.cols: " << covMatrix.cols() << endl;
            cout << "stateVec"<< stateVec << endl;
            cout << "covMatrix"<< covMatrix << endl;
        }*/

        double ratio = 0.0;//固定解的可靠性指标
        VectorXd stateVecFixed;//固定解的状态向量
        ARLambda arlambda;
        stateVecFixed = arlambda.resolve(stateVec, covMatrix);
        Vector3d dxyzFixed;
        dxyzFixed[0] = stateVecFixed[0];
        dxyzFixed[1] = stateVecFixed[1];
        dxyzFixed[2] = stateVecFixed[2];

        Vector3d xyzRTKFixed;
        xyzRTKFixed = xyzRover + dxyzFixed;

        // print fixed solution to files
        if (SYS == "G")printSolution(solStreamFixed, epoch, xyzRover, xyzRTKFixed);
        if (SYS == "C")printSolution(solStreamBDSFixed, epoch, xyzRover, xyzRTKFixed);


    }

    roverObsStream.close();
    baseObsStream.close();
    solStream.close();

}

