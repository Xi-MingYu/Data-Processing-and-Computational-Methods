#include <iostream>
#include "Eigen/Dense"
#include "Kalman_Filter.h"

using namespace Eigen;
using namespace std;

int main() {
    Eigen::VectorXd raw_data;
    raw_data.setRandom(2);
    cout << "raw data=" << raw_data << endl;
    Eigen::VectorXd result;
    Kalman_Filter Filter(STATE_DIMENSION, MEASURE_DIMENSION, CONTROL_DIMENSION);
    Filter.Init();
    result = Filter.Operate(raw_data);
    cout << "result=" << result << endl;
}
