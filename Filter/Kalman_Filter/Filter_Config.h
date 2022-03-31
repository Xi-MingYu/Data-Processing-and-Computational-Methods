//
// Created by 86150 on 2022/3/31.
//

#ifndef FILTER_FILTER_CONFIG_H
#define FILTER_FILTER_CONFIG_H


#define STATE_DIMENSION 2
#define MEASURE_DIMENSION 2
#define CONTROL_DIMENSION 2

///本部分有待完善，应将矩阵初始化失败纳入error处理
#define A_MATRIX_VALUE 1,1,0,1;
#define B_MATRIX_VALUE 0,0,0,0;
#define H_MATRIX_VALUE 1,0,0,1;
#define Q_MATRIX_VALUE 0.1,0,0,0.1;
#define R_MATRIX_VALUE 1,0,0,1;
#define P_MATRIX_INIT_VALUE 1,0,0,1;
#define x_VECTOR_INIT_VALUE 1,2;



#endif //FILTER_FILTER_CONFIG_H
