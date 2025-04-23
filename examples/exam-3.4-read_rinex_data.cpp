//
// Created by shjzh on 2025/2/24.
//

// 以函数方式读取文件；

#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include "GnssStruct.h"
#include "GnssFunc.h"
#include "TimeConvert.h"

using namespace std;

int main() {

    std::string rinexFile
            = "D:\\documents\\Source\\gnssLab-2024\\data\\Zero-baseline\\oem719-202203031500-1.obs"; // Replace with your actual RINEX file path

    cout << rinexFile << endl;

    std::fstream rinexFileStream(rinexFile);
    if (!rinexFileStream) {
        cerr << "rinex file open error!" << strerror(errno) << endl;
        exit(-1);
    }

//    RinexHeader rinexHeader;
//    parseRinexHeader(rinexFileStream,rinexHeader);

    CivilTime stopCivilTime = CivilTime(2022, 03, 03, 06, 48, 38.0000000);
    CommonTime stopEpoch = CivilTime2CommonTime(stopCivilTime);

    while (true) {
        ObsData obsData;

        // Read data from RINEX file
        try {
            obsData = (rinexFileStream);
            cout << "cs_detect_mw:" << endl;
            cout << obsData << endl;
        }
        catch (...) {
            break;
        }

        // 调试代码时，设置一个stopEpoch，有助于快速得到结果
        if (obsData.epoch > stopEpoch)
            break;

    }

    rinexFileStream.close();

    return 0;
}