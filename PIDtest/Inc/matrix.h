#ifndef MATRIX_H
#define MATRIX_H

//#include "port_type.h"

/*

    <-->：row列，： column行）



*/

typedef float MATRIX_TYPE_t;

#define Matrix(name,row,col) MATRIX_TYPE_t name[row*col]
typedef unsigned int UINT;

void Matrix_Init(MATRIX_TYPE_t* mat,UINT row,UINT col);
void eye_Matrix(MATRIX_TYPE_t* mat,UINT row);
void Matrix_Add(MATRIX_TYPE_t* lhs,MATRIX_TYPE_t* rhs, MATRIX_TYPE_t* result, UINT row, UINT col);
void Matrix_Sub(MATRIX_TYPE_t* lhs,MATRIX_TYPE_t* rhs, MATRIX_TYPE_t* result, UINT row, UINT col);
void Matrix_Scalar(MATRIX_TYPE_t* mat, MATRIX_TYPE_t _val, UINT row, UINT col);
void Matrix_Mul(MATRIX_TYPE_t* lhs,MATRIX_TYPE_t* rhs, MATRIX_TYPE_t* result, UINT row1, UINT col1row2 , UINT col2);
void Matrix_Tr(MATRIX_TYPE_t* mat, MATRIX_TYPE_t* result, UINT row, UINT col);
void Matrix_Inverse(MATRIX_TYPE_t* mat, MATRIX_TYPE_t* result, UINT dim);
void Matrix_copy(MATRIX_TYPE_t* mat1, MATRIX_TYPE_t* mat2, UINT row, UINT col);

void eulRot(MATRIX_TYPE_t* mat, float x, float y, float z);
void gyroRot(MATRIX_TYPE_t* mat, float dx, float dy, float dz);
void mat2ang(MATRIX_TYPE_t* mat, float *ax, float *ay, float *az);

void show_matrix(MATRIX_TYPE_t* mat,UINT row,UINT col);

#endif
