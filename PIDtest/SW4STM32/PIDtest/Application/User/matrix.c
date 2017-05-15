#include "matrix.h"
#include <stdio.h>
#include <math.h>
void Matrix_Init(MATRIX_TYPE_t* mat,UINT row,UINT col)
{
    UINT ix,elm_size = col*row;
    for(ix=0 ; ix<elm_size ; ix++){mat[ix]=0;}
}

void eye_Matrix(MATRIX_TYPE_t* mat,UINT row)
{
    Matrix_Init(mat, row, row);
    UINT i;
    for(i=0;i<row;i++)
        mat[i*row+i] = 1.0f;
}

void Matrix_Add(MATRIX_TYPE_t* lhs,MATRIX_TYPE_t* rhs, MATRIX_TYPE_t* result, UINT row, UINT col)
{
    UINT ix,elm_size = col*row;
    for(ix=0 ; ix<elm_size ; ix++){result[ix] = lhs[ix] + rhs[ix];}
}

void Matrix_Sub(MATRIX_TYPE_t* lhs,MATRIX_TYPE_t* rhs, MATRIX_TYPE_t* result, UINT row, UINT col)
{
    UINT ix,elm_size = col*row;
    for(ix=0 ; ix<elm_size ; ix++){result[ix] = lhs[ix] - rhs[ix];}
}

void Matrix_Scalar(MATRIX_TYPE_t* mat, MATRIX_TYPE_t _val, UINT row, UINT col)
{
    UINT ix,elm_size = col*row;
    for(ix=0 ; ix<elm_size ; ix++){mat[ix] = mat[ix]* _val;}
}

void Matrix_Mul(MATRIX_TYPE_t* lhs,MATRIX_TYPE_t* rhs, MATRIX_TYPE_t* result, UINT row1, UINT col1row2 , UINT col2)
{
    int i,j,k;
    //Matrix_Init(result, row1, col2);
    Matrix(temp,row1,col2);
    Matrix_Init(temp,row1,col2);
    for(i=0 ; i<row1 ; i++)
    {
        for(j=0 ; j<col2 ; j++)
        {
            for(k=0 ; k<col1row2 ; k++)
            {
                temp[i*col2+j]+=lhs[i*col1row2+k]*rhs[k*col2+j];
            }
        }
    }
    Matrix_copy(result, temp, row1, col2);
}

void Matrix_Tr(MATRIX_TYPE_t* mat, MATRIX_TYPE_t* result, UINT row, UINT col)
{
    int i,j;
    for(i=0 ; i<row ; i++)
    {
        for(j=0 ; j<col ; j++)
        {
            result[j*row+i] = mat[i*col+j];

        }
    }
}

void Matrix_Inverse(MATRIX_TYPE_t* mat, MATRIX_TYPE_t* result, UINT dim)
{

}

void Matrix_copy(MATRIX_TYPE_t* mat1, MATRIX_TYPE_t* mat2, UINT row, UINT col)
{
    UINT ix,elm_size = col*row;
    for(ix=0 ; ix<elm_size ; ix++){mat1[ix]=mat2[ix];}
}

void eulRot(MATRIX_TYPE_t* mat, float x, float y, float z)
{
    x = x*M_PI/180.0;
    y = y*M_PI/180.0;
    z = z*M_PI/180.0;
    mat[0] = cos(y)*cos(z);
    mat[1] = sin(x)*sin(y)*cos(z) - cos(x)*sin(z);
    mat[2] = cos(x)*sin(y)*cos(z) + sin(x)*sin(z);
    mat[3] = cos(y)*sin(z);
    mat[4] = sin(x)*sin(y)*sin(z) + cos(x)*cos(z);
    mat[5] = cos(x)*sin(y)*sin(z) - sin(x)*cos(z);
    mat[6] = -1 * sin(y);
    mat[7] = sin(x)*cos(y);
    mat[8] = cos(x)*cos(y);
}

void gyroRot(MATRIX_TYPE_t* mat, float dx, float dy, float dz)
{
    dx = dx*M_PI/180.0;
    dy = dy*M_PI/180.0;
    dz = dz*M_PI/180.0;
    Matrix(temp_M,3,3);
    Matrix(rot_M,3,3);
    eye_Matrix(temp_M,3);
    float temp_f[6];
    temp_M[1] = -dz; temp_M[2] = dy; temp_M[3] = dz;
    temp_M[5] = -dx; temp_M[6] = -dy; temp_M[7] = dx;
    Matrix_Mul(mat, temp_M, rot_M, 3, 3, 3);
    float error = .0f;
    UINT i;
    for(i=0;i<3;i++)
        error += rot_M[i]*rot_M[i+3];
    for(i=0;i<3;i++)
    {
        temp_f[i] = rot_M[i] - rot_M[i+3] * error / 2.0f;
        temp_f[i+3] = rot_M[i+3] - rot_M[i] * error / 2.0f;
    }
    for(i=0;i<6;i++)
        rot_M[i] = temp_f[i];
    rot_M[6] = rot_M[1]*rot_M[5] - rot_M[2]*rot_M[4];
    rot_M[7] = -rot_M[0]*rot_M[5] + rot_M[2]*rot_M[3];
    rot_M[8] = rot_M[0]*rot_M[4] - rot_M[1]*rot_M[3];
    for(i=0;i<3;i++)
        temp_f[i] = rot_M[i*3]*rot_M[i*3] + rot_M[i*3+1]*rot_M[i*3+1] + rot_M[i*3+2]*rot_M[i*3+2];
    for(i=0;i<3;i++)
    {
        rot_M[i] = (3.0f-temp_f[0])/2.0f*rot_M[i];
        rot_M[i+3] = (3.0f-temp_f[1])/2.0f*rot_M[i+3];
        rot_M[i+6] = (3.0f-temp_f[2])/2.0f*rot_M[i+6];
    }
    Matrix_copy(mat, rot_M, 3, 3);
}

void mat2ang(MATRIX_TYPE_t* mat, float *ax, float *ay, float *az)
{
    float temp_y1, temp_y2;
    *ay = *ay * M_PI / 180.0;
    if(mat[6] < 0.999f && mat[6] > -0.999f)
    {
        temp_y1 = -asin(mat[6]);
        temp_y2 = M_PI - temp_y1;
        if((*ay-temp_y1)*(*ay-temp_y1) > (*ay-temp_y2)*(*ay-temp_y2))
            *ay = temp_y2;
        else
            *ay = temp_y1;
        *ax = atan2((mat[7]/cos(*ay)),(mat[8]/cos(*ay)));
        *az = atan2((mat[3]/cos(*ay)),(mat[0]/cos(*ay)));
        *ax = *ax * 180.0/M_PI;
        *ay = *ay * 180.0/M_PI;
        *az = *az * 180.0/M_PI;
    }
    else
    {
        *az = .0f;
        if(mat[6] == -1.0f)
        {
            *ay = M_PI/2.0f;
            *ax = atan2(mat[1],mat[2]);
        }
        else
        {
            *ay = -M_PI/2.0f;
            *ax = atan2(-mat[1],-mat[2]);
        }
        *ax = *ax * 180.0/M_PI;
        *ay = *ay * 180.0/M_PI;
        *az = *az * 180.0/M_PI;
    }
}

void show_matrix(MATRIX_TYPE_t* mat,UINT row,UINT col)
{
    int ix,iy;
    for(ix=0 ; ix<row ; ix++)
    {
        for(iy=0 ; iy<col ; iy++)
        {
            printf("%.3f\t",mat[ix*col+iy]);
        }
        printf("\n");
    }
    printf("\n");
}
