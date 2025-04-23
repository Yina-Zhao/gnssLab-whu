//
// Created by shjzh on 2025/3/17.
//

#ifndef GNSSLAB_SOLVERKALMAN_H
#define GNSSLAB_SOLVERKALMAN_H

#include <Eigen/Eigen>
#include "GnssStruct.h"
#include "KalmanFilter.h"

using namespace Eigen;

//>>>>>>>>>>>>>>>>>
// 这个类的目的是把把方程系统和随机模型转变为矩阵形式，然后调用矩阵运算的KalmanFilter类实现卡尔曼滤波计算；
// 通过定义通用的方程系统，这个类可以用于单点定位，精密单点定位和rtk定位或者卫星定轨等任务
// todo:
// 考虑到kalman滤波与最小二乘很多模块是相同的，比如参数排序，获取参数解等等，因此可以设计为继承类
// 这样可以进一步减少代码的重复
//>>>>>>>>>>>>>>>>>>>
class SolverKalman {

public:
    SolverKalman() : firstTime(true) {};

    virtual void solve(EquSys &equSys, VariableDataMap& csData);
    void createIndex(const VariableSet &varSet );

    int getIndex(const VariableSet &varSet, const Variable &thisVar);
    double getSolution(const Parameter &type,
                       VariableSet &currentUnkSet,
                       const VectorXd &stateVec);

    VectorXd getState()
    {
        return solution;
    };

    MatrixXd getCovMatrix()
    {
        return covMatrix;
    };

    Eigen::Vector3d getdxyz() const{
        return dxyz;
    };

    /// Destructor.
    virtual ~SolverKalman() {};

private:

    bool firstTime;

    VectorXd solution, xhat;
    MatrixXd covMatrix, P;
    VectorXd postfitResidual;
    Vector3d dxyz;

    VariableSet currentUnkSet;
    VariableSet oldUnkSet;
    VariableIntMap currentIndexData;
    VariableIntMap oldIndexData;

    KalmanFilter kalmanFilter;

};


#endif //GNSSLAB_SOLVERKALMAN_H