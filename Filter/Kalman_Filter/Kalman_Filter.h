#ifndef FILTER_KALMAN_FILTER_H
#define FILTER_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>
#include "Filter_Config.h"

using namespace std;
using namespace Eigen;

class Kalman_Filter
{
public:
    Kalman_Filter(int state_dim_,
                  int measure_dim_,
                  int u_dim_);

public:
    void Init();
    Eigen::VectorXd Operate(Eigen::VectorXd x_measure);
    void StartParam_Init();
    void CovarMatrix_Init();
    void StateMatrix_Init();
    void Param_Construct(int state_dim, int measure_dim, int u_dim);

public:
    int state_dim; //state variable dimension
    int measure_dim; //measure variable dimension
    int u_dim; //control variable dimension
    Eigen::VectorXd x; //state variable
    Eigen::VectorXd z; //measure variable
    Eigen::VectorXd u; //control variable
    Eigen::MatrixXd A; //state matrix
    Eigen::MatrixXd B; //control matrix
    Eigen::MatrixXd P; //covariance matrix
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q; //process noise covariance matrix
    Eigen::MatrixXd R; //measurement noise covariance matrix


};


#endif //FILTER_KALMAN_FILTER_H
