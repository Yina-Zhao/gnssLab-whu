// Config.h
#pragma once

#include <string>

struct SPPConfigData {

    std::string obsFile;
    std::string navFile;
    std::string outFile;

    bool GPS;
    bool BD2;
    bool BD3;
    bool Galileo;
    bool GLONASS;

    int cutOffElevation;
    int minSatNum;
    int maxGDOP;
    int tropModel;
    int ionoModel;

    int obsModel;

    double noiseGPSCode;
    double noiseBD2Code;
    double noiseBD3Code;

    int estimator;
};

// 声明全局变量
extern SPPConfigData sppConfigData;

