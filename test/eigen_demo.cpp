#include <Eigen/Dense>
using namespace Eigen;

// View
const auto& Ab_ref = A.leftCols<6>();
// Ref
Eigen::Ref<Eigen::Matrix<SCALAR, 6, 6>> Ab_ref = A.leftCols<6>();

Matrix3d A;
Matrix3d v(1.0, 2.0, 3.0);
Matrix<double, 6, 1> b;

MatrixXd B(10,10);
VectorXd x(10);

A.setZero();
A.setIdentity();
A.setConstant(2.0);
A = Matrix3d::Random();

Matrix2d C;
C<< 1,2,
    3,4;

Matrix4d M = Matrix4d::Random();
auto R=M.block<3,3>(0,0);

auto r0=M.row(0);
auto c3=M.col(3);

VectorXd v(10);
auto v_head=v.head<3>();
auto v_tail=v.tail<4>();
auto v_segment=v.segment<4>(3);

Matrix2d A, B;
Vector2d v;

// 1. 矩阵乘法 & 向量乘法
Matrix2d C = A * B;
Vector2d y = A * v;

Matrix2d F = A.array() + 10.0;
Matrix2d G = A.array().square();

//MAP
double buffer[6] = {1, 2, 3, 4, 5, 6};

Map<VectorXd> v_map(buffer,6);
Map<MatrixXd> M_map(buffer,2,3);

MatrixXd A(100, 100), B(100, 100), C(100, 100);
C.noalias() = A*B;

