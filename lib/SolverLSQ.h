/**
 * @file SolverLSQ.hpp
 * Class to compute the Least Mean Squares Solution
 *
 * author
 *
 * shoujian zhang
 */

#ifndef SolverLSQ_HPP
#define SolverLSQ_HPP

#include <Eigen/Eigen>
#include "GnssStruct.h"

using namespace Eigen;

/* Class to compute the standard point positioning solution. By default,
 * this class only use GPS observables, if you want to use multi-gnss
 * observables, you should set them by using the method 'setSystems()'.
 */
class SolverLSQ {
public:


    /** Explicit constructor. Sets the default equation definition
     *  to be used when fed with GNSS data structures.
     */
    SolverLSQ() {};

    virtual void solve(EquSys &equSys);
    int getIndex(const VariableSet &varSet, const Variable &thisVar);
    double getSolution(const Parameter &type,
                       VariableSet &currentUnkSet,
                       const VectorXd &stateVec);

    Eigen::Vector3d getdxyz() const{
        return dxyz;
    };
    Eigen::VectorXd getState() const
    {
        return state;
    }
    Eigen::MatrixXd getCovMatrix() const
    {
        return covMatrix;
    }

    /// Destructor.
    virtual ~SolverLSQ() {};

private:

    VectorXd state;
    MatrixXd covMatrix;
    Vector3d dxyz;
    VariableSet currentUnkSet;
}; // End of class 'SolverLSQ'


#endif   // SolverLSQ_HPP
