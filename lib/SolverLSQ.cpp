/**
 * solver for spp using multi-gnss observation data
 *
 * shoujian zhang
 *
 * created by shoujianzhang, 2020.08.07
 * modify the input data from gnssRinex to Rx3ObsData, 2022.01.15
 *
 **/

#include <iomanip>
#include "SolverLSQ.h"
#include <fstream>

#define debug 1
using namespace std;


void SolverLSQ::solve(EquSys &equSys) {

    //if(debug)
     //   cout << "SolverLSQ:" << endl;

    currentUnkSet = equSys.varSet;
    int numUnk = currentUnkSet.size();
    int numObs = equSys.obsEquData.size();

    VectorXd prefit = VectorXd::Zero(numObs);
    MatrixXd hMatrix = MatrixXd::Zero(numObs, numUnk);
    MatrixXd wMatrix = MatrixXd::Zero(numObs, numObs);

    int iobs(0);
    for (auto ed: equSys.obsEquData) {
        prefit(iobs) = ed.second.prefit;

        for (auto vc: ed.second.varCoeffData) {
            // 从整体的X中搜索当前未知参数的位置
            int indexUnk = getIndex(currentUnkSet, vc.first);
            // 把偏导数插入到对应的h矩阵中
            hMatrix(iobs, indexUnk) = vc.second;
        }
        wMatrix(iobs, iobs) = ed.second.weight;

        iobs++;
    }

    MatrixXd hT = hMatrix.transpose();

    if (prefit.size()!= hMatrix.rows()) {
        InvalidSolver e("prefit size don't equal with rows of hMatrix");
        throw(e);
    }

    /*if (debug) {

        cout << "prefit:" <<endl;
        cout << prefit <<endl;

        cout << "hMatrix:" <<endl;
        cout << hMatrix <<endl;

        cout << "wMatrix:" <<endl;
        cout << wMatrix <<endl;
    }*/

    try {
        covMatrix = hT * wMatrix * hMatrix;
        covMatrix = covMatrix.inverse();
    }
    catch (...) {
        InvalidSolver e("Unable to invert matrix covMatrix");
        throw (e);
    }

    state = covMatrix * hT * wMatrix * prefit;

   /* if(debug)
    {
        cout << "state" << endl;
        cout << state.transpose() << endl;
    }*/

    double dx = getSolution(Parameter::dX, currentUnkSet, state);
    double dy = getSolution(Parameter::dY, currentUnkSet, state);
    double dz = getSolution(Parameter::dZ, currentUnkSet, state);

    dxyz[0] = dx;
    dxyz[1] = dy;
    dxyz[2] = dz;

}

int SolverLSQ::getIndex(const VariableSet &varSet, const Variable &thisVar) {
    int index(0);
    for (auto var: varSet) {
        if (var == thisVar) {
            break;
        }
        index++;
    }
    return index;
};

double SolverLSQ::getSolution(const Parameter &type,
                              VariableSet &currentUnkSet,
                              const VectorXd &stateVec)
noexcept(false) {
    // Declare an varIterator for 'stateMap' and go to the first element
    auto varIt = currentUnkSet.begin();
    int index(0);
    while ((*varIt).getParaType() != type) {
        // If the same type is not found, throw an exception
        if (varIt == currentUnkSet.end()) {
            InvalidRequest e("SolverLSQ::Type not found in state vector.");
            throw (e);
        }
        index++;
        varIt++;
    }

    // Else, return the corresponding value
    return stateVec(index);

}  // End of method 'SolverGeneral::getSolution()'




