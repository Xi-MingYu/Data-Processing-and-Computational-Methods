#include "Kalman_Filter.h"

Kalman_Filter::Kalman_Filter(int state_dim_,
                             int measure_dim_ =0,
                             int u_dim_=0)
                             : state_dim(state_dim_), measure_dim(measure_dim_), u_dim(u_dim_)
{
    if (state_dim == 0 || measure_dim == 0)
    {
        std::cerr << "state dimension and measure dimension should greater than 0";
    }
    Param_Construct(state_dim, measure_dim, u_dim);
}

void Kalman_Filter::Init()
{
    StateMatrix_Init();
    CovarMatrix_Init();
    StartParam_Init();
}

Eigen::VectorXd Kalman_Filter::Operate(Eigen::VectorXd z_measure)
{
    //单位阵
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);
    /**predict process**/
    x = A * x + B * u;
    P = A * P * A.transpose() + Q;
    /**correction process**/
    Eigen::MatrixXd tem = H * P * H.transpose() + R;
    Eigen::MatrixXd Kalman_Gain = P * H.transpose() * tem.inverse();
    x += Kalman_Gain * (z_measure - H * x);
    /**update process**/
    P= (I - Kalman_Gain * H ) * P;
    return x;
}


void Kalman_Filter::Param_Construct(int state_dim, int measure_dim, int u_dim)
{
    /**construct and initial**/
    x.resize(state_dim);
    x.setZero();
    A.resize(state_dim, state_dim);
    A.setIdentity();

    u.resize(u_dim);
    u.transpose();
    u.setZero();
    B.resize(state_dim, u_dim);
    B.setZero();

    z.resize(measure_dim);
    z.setZero();
    H.resize(measure_dim, state_dim);
    H.setZero();

    P.resize(state_dim, state_dim);
    P.setZero();
    Q.resize(state_dim, state_dim);
    Q.setZero();
    R.resize(measure_dim, measure_dim);
    R.setZero();
}

void Kalman_Filter::StateMatrix_Init()
{
    A << A_MATRIX_VALUE
    B << B_MATRIX_VALUE
    H << H_MATRIX_VALUE
}

void Kalman_Filter::StartParam_Init()
{
    x << x_VECTOR_INIT_VALUE
    P << P_MATRIX_INIT_VALUE
}

void Kalman_Filter::CovarMatrix_Init()
{
    Q << Q_MATRIX_VALUE
    R << R_MATRIX_VALUE
}