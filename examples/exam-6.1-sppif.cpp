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
#include "SPPIFCode.h"

#define debug 1

using namespace std;

int main() {

    //--------------------
    // 打开文件流
    //--------------------

    // Replace with your actual RINEX file path
    string dirPath = "D:\\documents\\Source\\gnssLab-2.2\\data\\Zero-baseline\\";
//    string dirPath = "D:\\documents\\Source\\gnssLab-2.1\\data\\";

    // rover obs file name
    std::string roverFile = dirPath + "oem719-202203031500-1.obs";
    //std::string roverFile = dirPath + "ABMF00GLP_R_20210010000_01D_30S_MO.rnx";

    cout << roverFile << endl;

    // nav file name, download from IGS ftp site:ftp://gssc.esa.int/gnss/data/daily/YYYY/brdc
    std::string navFile = dirPath + "BRDC00IGS_R_20220620000_01D_MN.rnx";
   // std::string navFile = dirPath + "ABMF00GLP_R_20210010000_01D_MN.rnx";

    // 2022 03 03 06 48 37.0000000  0 45
    CivilTime stopCivilTime = CivilTime(2022, 03, 03, 06, 48 , 37);
    //   CivilTime stopCivilTime = CivilTime(2021, 01, 01, 10, 00 , 00);

    CommonTime stopEpoch = CivilTime2CommonTime(stopCivilTime);

    std::fstream roverObsStream(roverFile);
    if (!roverObsStream) {
        cerr << "rover file open error!" << strerror(errno) << endl;
        exit(-1);
    }
    
    // read nav file data before rtk
    RinexNavStore navStore;
    navStore.loadFile(navFile);

    cout << "after NavStore" << endl;

    std::map<string, std::set<string>> selectedTypes;
    selectedTypes["G"].insert("C1C");
    selectedTypes["G"].insert("C2W");
    selectedTypes["G"].insert("L1C");
    selectedTypes["G"].insert("L2W");


    std::map<string, std::pair<string, string>> ifCodeTypes;
    ifCodeTypes["G"].first = "C1";
    ifCodeTypes["G"].second = "C2";

    //-------------------
    // 定义数据处理的对象
    //-------------------
    //>>> classes for rover
    RinexObsReader readObsRover;
    readObsRover.setFileStream(&roverObsStream);
    readObsRover.setSelectedTypes(selectedTypes);

    SPPIFCode sppif;
    sppif.setRinexNavStore(&navStore);
    sppif.setIFCodeTypes(ifCodeTypes);
    
    //>>> classes for rtk;    
    SolverLSQ solver;


    std::string solFile = roverFile + ".spp.out";
    if(debug)
        cout << solFile << endl;

    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        cerr << "solution file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    while (true) {

        // solve spp for rover
        ObsData roverData;

        try {
            roverData = readObsRover.parseRinexObs();
            cout << "roverData:" << roverData << endl;
        }
        catch (EndOfFile &e) { break; }

        CommonTime epoch = roverData.epoch;

        // 最小二乘
        sppif.solve(roverData);
        EquSys equSysRover = sppif.getEquSys();
        Vector3d xyzRover = sppif.getXYZ();

        cout << "sppif:" << CommonTime2YDSTime(epoch) << xyzRover.transpose() << endl;

        // print solution to files
        printSolution(solStream, epoch, xyzRover);

        // 调试代码时，设置一个stopEpoch，有助于快速得到结果
        if (roverData.epoch > stopEpoch)
            break;

    }

    roverObsStream.close();
    solStream.close();

}

