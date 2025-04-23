//
// Created by shjzh on 2025/2/25.
//

#ifndef GNSSLAB_GNSSFUNC_H
#define GNSSLAB_GNSSFUNC_H

#include <string>
#include <set>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include "CoordStruct.h"
#include "GnssStruct.h"
#include "StringUtils.h"

using namespace Eigen;

// Changing Sign
inline double sign(double x)
{
    return (x <= 0.0) ? -1.0 : 1.0;
};

// Rounding Values
inline double round(double x)
{
    return double(std::floor(x + 0.5));
};

// Swapping values
inline void swap(double& a, double& b)
{
    double t(a); a = b; b = t;
};

void parseRinexHeader(std::fstream &rinexFileStream,
                      RinexHeader &rinexHeader);

ObsData parseRinexObs(std::fstream &rinexFileStream);

CommonTime parseTime(const string &line);

void chooseObs(ObsData &obsData,
               std::map<string, std::set<string>> &sysTypes);

void convertObsType(ObsData &obsData);

double wavelengthOfMW(string sys,
                      string L1Type,
                      string L2Type);

double varOfMW(string,
               string L1Type,
               string L2Type);

void detectCSMW(ObsData &obsData,
                std::map<Variable, int> &csFlagData,
                SatEpochValueMap &satEpochMWData,
                SatEpochValueMap &satEpochMeanMWData,
                SatEpochValueMap &satEpochCSFlagData);

void differenceStation(EquSys& equSysRover, VariableDataMap& csFlagRover,
                       EquSys& equSysBase, VariableDataMap& csFlagBase,
                       EquSys& equSysSD, VariableDataMap& csFlagSD);


void differenceStation(EquSys& equSysRover,
                       EquSys& equSysBase,
                       EquSys& equSysSD);

SatID findDatumSat(bool& firstEpoch,
                   SatValueMap& satElevData);

void differenceSat( SatID& datumSat,
                    EquSys& equSysSD, VariableDataMap& csFlagSD,
                    EquSys& equSysDD, VariableDataMap& csFlagDD);

void differenceSat( SatID& datumSat,
                    EquSys& equSysSD,
                    EquSys& equSysDD);

void ambiguityDatum(bool& firstEpoch,
                    SatID& datumSat,
                    VariableDataMap& fixedAmbData,
                    EquSys& equSysDD);

void fixSolution(VectorXd& stateVec,
                 MatrixXd& covMatrix,
                 VariableSet& varSet,
                 double& ratio,
                 Vector3d& dxyzFixed,
                 VariableDataMap& fixedAmbData);

// print solution to files
void printSolution(std::fstream & solStream,
                   CommonTime& ctTime,
                   Eigen::Vector3d& xyzRover,
                   Eigen::Vector3d& xyzRTKFloat,
                   double& ratio,
                   Eigen::Vector3d& xyzRTKFixed);

// print solution to files
void printSolution(std::fstream & solStream,
                   CommonTime& ctTime,
                   Eigen::Vector3d& xyzRover,
                   Eigen::Vector3d& xyzRTKFloat);

// print solution to files
void printSolution(std::fstream & solStream,
                   CommonTime& ctTime,
                   Eigen::Vector3d& xyzRover);

#endif //GNSSLAB_GNSSFUNC_H
